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

#include "platform.h"

#ifdef USE_FENCE

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fence.h"

PG_REGISTER_WITH_RESET_TEMPLATE(fenceConfig_t, fenceConfig, PG_FENCE_CONFIG, 1);

PG_RESET_TEMPLATE(fenceConfig_t, fenceConfig,
    .altitudeMode = FENCE_MODE_OFF,
    .maxAltitudeM = 120,            // open category ceiling
    .altitudeWarningM = 20,
    .distanceMode = FENCE_MODE_OFF,
    .maxDistanceM = 1000,
    .distanceWarningM = 20,
    .brakeAngleDeg = 30,
);

#endif // USE_FENCE
