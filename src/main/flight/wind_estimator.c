/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#if defined(USE_WIND_ESTIMATOR)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "navigation/navigation_pos_estimator_private.h"

#include "io/gps.h"

// Based on WindEstimation.pdf paper
#define WINDESTIMATOR_TIMEOUT       60*15 // 15min with out altitude change
#define WINDESTIMATOR_ALTITUDE_SCALE WINDESTIMATOR_TIMEOUT/500.0f //or 500m altitude change

#define WINDESTIMATOR_VALIDITY_THRESHOLD    15
#define WINDESTIMATOR_SPIKE_FILTER_ADJ_FACTOR   50

static bool hasValidWindEstimate = false;
static float estimatedWind[XYZ_AXIS_COUNT] = {0, 0, 0};    // wind velocity vectors in cm / sec in earth frame
static float lastGroundVelocity[XYZ_AXIS_COUNT];
static float lastFuselageDirection[XYZ_AXIS_COUNT];

bool isEstimatedWindSpeedValid(void)
{
    return hasValidWindEstimate
#ifdef USE_GPS_FIX_ESTIMATION
        || STATE(GPS_ESTIMATED_FIX)  //use any wind estimate with GPS fix estimation.
#endif
        ;
}

float getEstimatedWindSpeed(int axis)
{
    return estimatedWind[axis];
}

float getEstimatedHorizontalWindSpeed(uint16_t *angle)
{
    float xWindSpeed = getEstimatedWindSpeed(X);
    float yWindSpeed = getEstimatedWindSpeed(Y);
    if (angle) {
        float horizontalWindAngle = atan2_approx(yWindSpeed, xWindSpeed);
        // atan2 returns [-M_PI, M_PI], with 0 indicating the vector points in the X direction
        // We want [0, 360) in degrees
        if (horizontalWindAngle < 0) {
            horizontalWindAngle += 2 * M_PIf;
        }
        *angle = RADIANS_TO_CENTIDEGREES(horizontalWindAngle);
    }
    return calc_length_pythagorean_2D(xWindSpeed, yWindSpeed);
}

void updateWindEstimator(timeMs_t currentTimeMs)
{
    static timeMs_t lastUpdateMs = 0;       // CR172
    if (currentTimeMs - lastUpdateMs < 1000) {
        return;
    }
    lastUpdateMs = currentTimeMs;

    static timeMs_t lastUseableAttitudeUpdateMs = 0;
    static timeMs_t lastValidWindEstimateMs = 0;
    static float lastValidEstimateAltitude = 0.0f;
    static uint8_t validityScore = 0;
    static uint8_t spikeFilterDynAdjustment = WINDESTIMATOR_SPIKE_FILTER_ADJ_FACTOR;
    static bool initialEstimate = true;

    float currentAltitude = gpsSol.llh.alt / 100.0f; // altitude in m

    if ((MS2S(currentTimeMs - lastValidWindEstimateMs) + WINDESTIMATOR_ALTITUDE_SCALE * fabsf(currentAltitude - lastValidEstimateAltitude)) > WINDESTIMATOR_TIMEOUT) {
        hasValidWindEstimate = false;
        validityScore = 0;      // CR172
    }

    if (!validityScore) {
        hasValidWindEstimate = false;
    } else if (!hasValidWindEstimate && validityScore > WINDESTIMATOR_VALIDITY_THRESHOLD) {
        hasValidWindEstimate = true;
    }
    // DEBUG_SET(DEBUG_ALWAYS, 4, spikeFilterDynAdjustment);
    // DEBUG_SET(DEBUG_ALWAYS, 5, validityScore);
    DEBUG_SET(DEBUG_ALWAYS, 0, 100);
    if (!isGPSHeadingValid() || !gpsSol.flags.validVelNE || !gpsSol.flags.validVelD
#ifdef USE_GPS_FIX_ESTIMATION
        || STATE(GPS_ESTIMATED_FIX)
#endif
        ) {
        return;
    }
    DEBUG_SET(DEBUG_ALWAYS, 0, 200);
    float groundVelocity[XYZ_AXIS_COUNT];
    float groundVelocityDiff[XYZ_AXIS_COUNT];
    float groundVelocitySum[XYZ_AXIS_COUNT];

    float fuselageDirection[XYZ_AXIS_COUNT];
    float fuselageDirectionDiff[XYZ_AXIS_COUNT];
    float fuselageDirectionSum[XYZ_AXIS_COUNT];

    // Get current 3D velocity from GPS in cm/s relative to earth frame
    groundVelocity[X] = posEstimator.gps.vel.x;
    groundVelocity[Y] = posEstimator.gps.vel.y;
    groundVelocity[Z] = posEstimator.gps.vel.z;

    // Fuselage direction in earth frame rads ?
    fuselageDirection[X] = HeadVecEFFiltered.x;
    fuselageDirection[Y] = -HeadVecEFFiltered.y;
    // fuselageDirection[Z] = -HeadVecEFFiltered.z;
    fuselageDirection[Z] = HeadVecEFFiltered.z;

    if (MS2S(currentTimeMs - lastUseableAttitudeUpdateMs) > 10 || lastUseableAttitudeUpdateMs == 0) {
        lastUseableAttitudeUpdateMs = currentTimeMs;
        memcpy(lastFuselageDirection, fuselageDirection, sizeof(lastFuselageDirection));
        memcpy(lastGroundVelocity, groundVelocity, sizeof(lastGroundVelocity));
        return;
    }
    DEBUG_SET(DEBUG_ALWAYS, 0, 300);
    fuselageDirectionDiff[X] = fuselageDirection[X] - lastFuselageDirection[X];
    fuselageDirectionDiff[Y] = fuselageDirection[Y] - lastFuselageDirection[Y];
    fuselageDirectionDiff[Z] = fuselageDirection[Z] - lastFuselageDirection[Z];

    float diffLengthSq = sq(fuselageDirectionDiff[X]) + sq(fuselageDirectionDiff[Y]) + sq(fuselageDirectionDiff[Z]);
        DEBUG_SET(DEBUG_ALWAYS, 7, 1000 * diffLengthSq);
    // Very small changes in attitude will result in a denominator
    // very close to zero which will introduce too much error in the
    // estimation.

    // TODO: Is 0.2f an adequate threshold?
    if (diffLengthSq > sq(0.2f)) {
        DEBUG_SET(DEBUG_ALWAYS, 0, 400);
        lastUseableAttitudeUpdateMs = currentTimeMs;

        // when turning, use the attitude response to estimate wind speed
        groundVelocityDiff[X] = groundVelocity[X] - lastGroundVelocity[X];
        groundVelocityDiff[Y] = groundVelocity[Y] - lastGroundVelocity[Y];
        groundVelocityDiff[Z] = groundVelocity[Z] - lastGroundVelocity[Z];

        // estimate airspeed using equation 6
        float V = (calc_length_pythagorean_3D(groundVelocityDiff[X], groundVelocityDiff[Y], groundVelocityDiff[Z])) / fast_fsqrtf(diffLengthSq);

        fuselageDirectionSum[X] = fuselageDirection[X] + lastFuselageDirection[X];
        fuselageDirectionSum[Y] = fuselageDirection[Y] + lastFuselageDirection[Y];
        fuselageDirectionSum[Z] = fuselageDirection[Z] + lastFuselageDirection[Z];

        groundVelocitySum[X] = groundVelocity[X] + lastGroundVelocity[X];
        groundVelocitySum[Y] = groundVelocity[Y] + lastGroundVelocity[Y];
        groundVelocitySum[Z] = groundVelocity[Z] + lastGroundVelocity[Z];

        memcpy(lastFuselageDirection, fuselageDirection, sizeof(lastFuselageDirection));
        memcpy(lastGroundVelocity, groundVelocity, sizeof(lastGroundVelocity));

        float theta = atan2f(groundVelocityDiff[Y], groundVelocityDiff[X]) - atan2f(fuselageDirectionDiff[Y], fuselageDirectionDiff[X]);// equation 9
        float sintheta = sinf(theta);
        float costheta = cosf(theta);

        float wind[XYZ_AXIS_COUNT];
        wind[X] = (groundVelocitySum[X] - V * (costheta * fuselageDirectionSum[X] - sintheta * fuselageDirectionSum[Y])) * 0.5f;// equation 10
        wind[Y] = (groundVelocitySum[Y] - V * (sintheta * fuselageDirectionSum[X] + costheta * fuselageDirectionSum[Y])) * 0.5f;// equation 11
        wind[Z] = (groundVelocitySum[Z] - V * fuselageDirectionSum[Z]) * 0.5f;// equation 12

        DEBUG_SET(DEBUG_ALWAYS, 1, wind[X]);
        DEBUG_SET(DEBUG_ALWAYS, 2, wind[Y]);
        DEBUG_SET(DEBUG_ALWAYS, 3, wind[Z]);
        // DEBUG_SET(DEBUG_ALWAYS, 6, initialEstimate);

        static uint8_t spikeFilterResetCounter = 0;     // CR172
        // DEBUG_SET(DEBUG_ALWAYS, 6, spikeFilterResetCounter);

        if (initialEstimate) {
            if (validityScore == 2 * WINDESTIMATOR_VALIDITY_THRESHOLD) {    // CR172
                initialEstimate = false;
                spikeFilterDynAdjustment = 0;
            }
        } else if (spikeFilterDynAdjustment || spikeFilterResetCounter > 30) {
            if (spikeFilterDynAdjustment < WINDESTIMATOR_SPIKE_FILTER_ADJ_FACTOR) {
                spikeFilterDynAdjustment++;
                if (hasValidWindEstimate && validityScore > 0) validityScore--;
            }
            spikeFilterResetCounter = 0;        // CR172
        } else {
            spikeFilterResetCounter++;   // CR172
        }

        uint16_t spikeFilterThreshold = 500 + spikeFilterDynAdjustment * WINDESTIMATOR_SPIKE_FILTER_ADJ_FACTOR;   // CR172
        for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            if (ABS(wind[axis] - estimatedWind[axis]) > (spikeFilterThreshold)) {
                return;
            }
        }
DEBUG_SET(DEBUG_ALWAYS, 0, 500);
        float filterAlpha = 50.0f / spikeFilterThreshold;  // CR172
        estimatedWind[X] = estimatedWind[X] + filterAlpha * (wind[X] - estimatedWind[X]);
        estimatedWind[Y] = estimatedWind[Y] + filterAlpha * (wind[Y] - estimatedWind[Y]);
        estimatedWind[Z] = estimatedWind[Z] + filterAlpha * (wind[Z] - estimatedWind[Z]);

        if (validityScore < 2 * WINDESTIMATOR_VALIDITY_THRESHOLD) validityScore++;

        if (spikeFilterDynAdjustment) {
            spikeFilterDynAdjustment = MAX(0, spikeFilterDynAdjustment - (initialEstimate ? 1 : 3));
        }

        lastValidWindEstimateMs = currentTimeMs;
        lastValidEstimateAltitude = currentAltitude;
        spikeFilterResetCounter = 0;  // CR172
    }
}

#endif
