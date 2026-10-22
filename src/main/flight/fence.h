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

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#include "pg/fence.h"

// Flight envelope fence: an altitude ceiling above the arming point and a horizontal
// radius around home, each either warned about or enforced.
//
// The ceiling is enforced in every flight mode by capping the throttle to what still
// lets the aircraft stop below it, and by keeping the altitude targets of the autopilot
// (alt hold, pos hold, rescue, missions) under it. The pilot can always throttle down.
//
// The radius is enforced by taking the aircraft as soon as it could no longer stop
// inside it: position hold brakes it, flies it back inside, and holds it there until
// the pilot centres the roll and pitch sticks, which hands control back.

typedef enum {
    FENCE_WARNING_NONE = 0,
    FENCE_WARNING_RETURNING,        // the fence is flying the aircraft back inside the radius
    FENCE_WARNING_HOLDING,          // ...and holds it there until the sticks are centred
    FENCE_WARNING_ALTITUDE_MAX,     // at or above the ceiling
    FENCE_WARNING_DISTANCE_MAX,     // at or beyond the radius
    FENCE_WARNING_UNAVAILABLE,      // a limit is set but cannot be measured or enforced right now
    FENCE_WARNING_ALTITUDE_NEAR,    // within the warning margin of the ceiling
    FENCE_WARNING_DISTANCE_NEAR,    // within the warning margin of the radius
} fenceWarning_e;

#ifdef USE_FENCE

extern const char * const fenceModeNames[FENCE_MODE_COUNT];

void fenceInit(void);
// TASK_POSITION, after the estimator has run
void fenceUpdate(timeUs_t currentTimeUs);

// Arming is refused while a limit is enforced but cannot be measured
bool fenceIsReadyToArm(void);

// Ceiling: the pilot's or the autopilot's throttle, capped (mixer, every PID loop)
float fenceLimitThrottle(float throttle);
// Ceiling: lowers an autopilot altitude target (cm, estimator frame) to the ceiling.
// Returns true when it did.
bool fenceConstrainAltitudeTarget(float *targetAltitudeCm);

// Radius: the fence has taken the aircraft (angle, alt hold and pos hold are forced)
bool fenceOwnsFlight(void);
// Radius: the fence is braking or flying back, so the pilot's roll and pitch are ignored
bool fenceOwnsPosition(void);
// Radius: the limit autopilot missions are held to as well, metres, 0 = none
uint16_t fenceGetDistanceLimitM(void);

fenceWarning_e fenceGetWarning(void);
// Metres left to the ceiling / radius, for the warnings
int fenceGetAltitudeMarginM(void);
int fenceGetDistanceMarginM(void);

#endif // USE_FENCE
