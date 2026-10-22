/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "pg/pg.h"

#define FENCE_ALTITUDE_MIN_M        10
#define FENCE_ALTITUDE_MAX_M        1000
#define FENCE_DISTANCE_MIN_M        20
#define FENCE_DISTANCE_MAX_M        10000
#define FENCE_WARNING_MARGIN_MAX_M  100

// What a fence does about the limit it guards
typedef enum {
    FENCE_MODE_OFF = 0,     // not checked
    FENCE_MODE_WARN,        // OSD warnings only, the pilot keeps full control
    FENCE_MODE_LIMIT,       // warnings, arming checks, and the aircraft is held inside the limit
    FENCE_MODE_COUNT
} fenceMode_e;

typedef struct fenceConfig_s {
    uint8_t altitudeMode;           // fenceMode_e
    uint16_t maxAltitudeM;          // ceiling above the arming point, metres
    uint8_t altitudeWarningM;       // warn this far below the ceiling, metres
    uint8_t distanceMode;           // fenceMode_e
    uint16_t maxDistanceM;          // horizontal radius around home, metres
    uint8_t distanceWarningM;       // warn this far inside the radius, and return at least this far inside it, metres
} fenceConfig_t;

PG_DECLARE(fenceConfig_t, fenceConfig);
