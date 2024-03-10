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

#include "common/bitarray.h"

#include "config/parameter_group.h"

#define BOXID_NONE 255

// typedef enum {
    // BOXARM           = 0,
    // BOXANGLE         = 1,
    // BOXHORIZON       = 2,
    // BOXNAVALTHOLD    = 3,    // old BOXBARO
    // BOXHEADINGHOLD   = 4,    // old MAG
    // BOXHEADFREE      = 5,
    // BOXHEADADJ       = 6,
    // BOXCAMSTAB       = 7,
    // BOXNAVRTH        = 8,    // old GPSHOME
    // BOXNAVPOSHOLD    = 9,    // old GPSHOLD
    // BOXMANUAL        = 10,
    // BOXBEEPERON      = 11,
    // BOXLEDLOW        = 12,
    // BOXLIGHTS        = 13,
    // BOXNAVLAUNCH     = 14,
    // BOXOSD           = 15,
    // BOXTELEMETRY     = 16,
    // BOXBLACKBOX      = 17,
    // BOXFAILSAFE      = 18,
    // BOXNAVWP         = 19,
    // BOXAIRMODE       = 20,
    // BOXHOMERESET     = 21,
    // BOXGCSNAV        = 22,
    // BOXKILLSWITCH    = 23,   // old HEADING LOCK
    // BOXSURFACE       = 24,
    // BOXFLAPERON      = 25,
    // BOXTURNASSIST    = 26,
    // BOXAUTOTRIM      = 27,
    // BOXAUTOTUNE      = 28,
    // BOXCAMERA1       = 29,
    // BOXCAMERA2       = 30,
    // BOXCAMERA3       = 31,
    // BOXOSDALT1       = 32,
    // BOXOSDALT2       = 33,
    // BOXOSDALT3       = 34,
    // BOXNAVCOURSEHOLD = 35,
    // BOXBRAKING       = 36,
    // BOXUSER1         = 37,
    // BOXUSER2         = 38,
    // BOXFPVANGLEMIX   = 39,
    // BOXLOITERDIRCHN  = 40,
    // BOXMSPRCOVERRIDE = 41,
    // BOXPREARM        = 42,
    // BOXTURTLE        = 43,
    // BOXNAVCRUISE     = 44,
    // BOXAUTOLEVEL     = 45,
    // BOXPLANWPMISSION = 46,
    // BOXSOARING       = 47,
    // BOXUSER3         = 48,
    // BOXUSER4         = 49,
    // BOXCHANGEMISSION = 50,
    // BOXBEEPERMUTE    = 51,
    // BOXMULTIFUNCTION = 52,
    // BOXANGLEHOLD     = 53,
    // CHECKBOX_ITEM_COUNT
// } boxId_e;

typedef enum {
    BOXARM           = 0,
    BOXANGLE         = 1,
    BOXHORIZON       = 2,
    BOXANGLEHOLD     = 3,
    BOXMANUAL        = 4,
    BOXNAVALTHOLD    = 5,    // old BOXBARO 3
    BOXHEADINGHOLD   = 6,    // old MAG  4
    BOXNAVPOSHOLD    = 7,    // old GPSHOLD  9
    BOXNAVCOURSEHOLD = 8,
    BOXNAVCRUISE     = 9,
    BOXNAVRTH        = 10,    // old GPSHOME  8
    BOXNAVWP         = 11,
    BOXNAVLAUNCH     = 12,
    BOXSOARING       = 13,
    BOXFAILSAFE      = 14,
    BOXAUTOLEVEL     = 15,
    BOXHEADFREE      = 16,
    BOXHEADADJ       = 17,
    BOXCAMSTAB       = 18,
    BOXBEEPERON      = 19,
    BOXAUTOTRIM      = 20,
    BOXAUTOTUNE      = 21,
    BOXOSD           = 22,
    BOXTELEMETRY     = 23,
    BOXBLACKBOX      = 24,
    BOXAIRMODE       = 25,
    BOXHOMERESET     = 26,
    BOXGCSNAV        = 27,
    BOXKILLSWITCH    = 28,   // old HEADING LOCK  23
    BOXSURFACE       = 29,
    BOXFLAPERON      = 30,
    BOXTURNASSIST    = 31,
    BOXLEDLOW        = 32,
    BOXLIGHTS        = 33,
    BOXCAMERA1       = 34,
    BOXCAMERA2       = 35,
    BOXCAMERA3       = 36,
    BOXOSDALT1       = 37,
    BOXOSDALT2       = 38,
    BOXOSDALT3       = 39,
    BOXBRAKING       = 40,
    BOXUSER1         = 41,
    BOXUSER2         = 42,
    BOXFPVANGLEMIX   = 43,
    BOXLOITERDIRCHN  = 44,
    BOXMSPRCOVERRIDE = 45,
    BOXPREARM        = 46,
    BOXTURTLE        = 47,
    BOXPLANWPMISSION = 48,
    BOXUSER3         = 49,
    BOXUSER4         = 50,
    BOXCHANGEMISSION = 51,
    BOXBEEPERMUTE    = 52,
    BOXMULTIFUNCTION = 53,
    BOXMIXERPROFILE      = 54,
    BOXMIXERTRANSITION   = 55,
    CHECKBOX_ITEM_COUNT
} boxId_e;

// type to hold enough bits for CHECKBOX_ITEM_COUNT. Struct used for value-like behavior
typedef struct boxBitmask_s { BITARRAY_DECLARE(bits, CHECKBOX_ITEM_COUNT); } boxBitmask_t;

#define MAX_MODE_ACTIVATION_CONDITION_COUNT 40

#define CHANNEL_RANGE_MIN 900
#define CHANNEL_RANGE_MAX 2100

#define CHANNEL_RANGE_STEP_WIDTH 25

#define MODE_STEP_TO_CHANNEL_VALUE(step) (CHANNEL_RANGE_MIN + CHANNEL_RANGE_STEP_WIDTH * step)
#define CHANNEL_VALUE_TO_STEP(channelValue) ((constrain(channelValue, CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX) - CHANNEL_RANGE_MIN) / CHANNEL_RANGE_STEP_WIDTH)

#define MIN_MODE_RANGE_STEP 0
#define MAX_MODE_RANGE_STEP ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / CHANNEL_RANGE_STEP_WIDTH)

#define IS_RANGE_USABLE(range) ((range)->startStep < (range)->endStep)

// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 1200
typedef struct channelRange_s {
    uint8_t startStep;
    uint8_t endStep;
} channelRange_t;

typedef struct modeActivationCondition_s {
    boxId_e modeId;
    uint8_t auxChannelIndex;
    channelRange_t range;
} modeActivationCondition_t;

typedef enum {
    MODE_OPERATOR_OR, // default
    MODE_OPERATOR_AND
} modeActivationOperator_e;

typedef struct modeActivationOperatorConfig_s {
    modeActivationOperator_e modeActivationOperator;
} modeActivationOperatorConfig_t;

PG_DECLARE_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions);
PG_DECLARE(modeActivationOperatorConfig_t, modeActivationOperatorConfig);

bool IS_RC_MODE_ACTIVE(boxId_e boxId);
void rcModeUpdate(boxBitmask_t *newState);

bool isModeActivationConditionPresent(boxId_e modeId);

void processAirmode(void);
bool isUsingNavigationModes(void);
bool isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range);

void updateActivatedModes(void);
void updateUsedModeActivationConditionFlags(void);
