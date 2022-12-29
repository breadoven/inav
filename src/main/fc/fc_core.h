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

#pragma once

#include <stdbool.h>

#include "common/time.h"

typedef enum disarmReason_e {
    DISARM_NONE         = 0,
    DISARM_TIMEOUT      = 1,
    DISARM_STICKS       = 2,
    DISARM_SWITCH_3D    = 3,
    DISARM_SWITCH       = 4,
    DISARM_KILLSWITCH   = 5,
    DISARM_FAILSAFE     = 6,
    DISARM_NAVIGATION   = 7,
    DISARM_LANDING      = 8,
    DISARM_REASON_COUNT
} disarmReason_t;
// // cr88
// typedef enum {
    // MULTI_FUNC_NONE,
    // MULTI_FUNC_1,
    // MULTI_FUNC_2,
    // MULTI_FUNC_3,
    // MULTI_FUNC_COUNT,
// } multi_function_e;

// bool multiFunctionSelection(multi_function_e * returnItem);
// // cr88
void handleInflightCalibrationStickPosition(void);

void disarm(disarmReason_t disarmReason);
timeUs_t getLastDisarmTimeUs(void);
void tryArm(bool forceArm);     // CR88
disarmReason_t getDisarmReason(void);

bool emergencyArmingUpdate(bool armingSwitchIsOn);  // CR86
bool emergencyArmingCanOverrideArmingDisabled(void); // CR88

bool areSensorsCalibrating(void);
float getFlightTime(void);
float getArmTime(void);

void fcReboot(bool bootLoader);