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

#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_FENCE

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/flight_plan_nav.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/pos_hold.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "flight/position_nav.h"

#include "io/gps.h"

#include "pg/autopilot.h"
#include "pg/gps_rescue.h"
#include "pg/pos_hold.h"

#include "rx/rx.h"

#include "fence.h"

// Ceiling. The throttle is capped to what holds the climb rate to an envelope that still
// stops the aircraft at the ceiling, so the cap only bites when the aircraft could not
// otherwise stop in time, and never raises the throttle.
#define FENCE_ALT_BRAKE_DECEL_CMSS      500.0f      // vertical deceleration the envelope plans on; an idle throttle gives well over this
#define FENCE_ALT_POSITION_GAIN         1.0f        // 1/s: climb rate allowed per cm below the ceiling close to it, descent rate per cm above it
#define FENCE_ALT_RECOVERY_RATE_CMS     200.0f      // fastest the ceiling pushes the aircraft back down
#define FENCE_ALT_RATE_GAIN             0.001f      // throttle per cm/s of climb rate over the envelope
#define FENCE_ALT_RATE_I_GAIN           0.0005f     // throttle per cm climbed over the envelope, while the cap is flying the aircraft
#define FENCE_ALT_RATE_I_LIMIT          0.15f
#define FENCE_ALT_RATE_I_DECAY          2.0f        // 1/s: the integral bleeds away once the pilot is below the cap again
#define FENCE_ALT_TARGET_MARGIN_CM      200.0f      // autopilot altitude targets stay this far below the ceiling
#define FENCE_ALT_CAP_LPF_HZ            10.0f       // smooths the cap's 100 Hz steps and the climb rate noise it follows

// Throttle handover: the fence eases the throttle over to alt hold when it takes the aircraft,
// and back to the pilot's stick when it hands it back, rather than stepping it
#define FENCE_TAKEOVER_TIME_S           0.3f
#define FENCE_HANDBACK_TIME_S           1.0f

// Radius
#define FENCE_DIST_BRAKE_EFFICIENCY     0.5f        // of g * tan(brake angle), the deceleration the braking is counted on for
#define FENCE_DIST_RETURN_MARGIN_MIN_M  10          // the return point is at least this far inside the radius
#define FENCE_DIST_STOPPED_SPEED_CMS    100.0f      // braking is over below this ground speed
#define FENCE_DIST_BRAKING_TIMEOUT_US   5000000     // ...or after this long
#define FENCE_DIST_RETURN_TIMEOUT_US    60000000    // a return still not there by then hands the aircraft back
#define FENCE_DIST_ARRIVAL_RADIUS_M     2.0f
#define FENCE_DIST_ARRIVAL_SPEED_MPS    0.5f
#define FENCE_DIST_SETTLE_TIMEOUT_S     3.0f
#define FENCE_DIST_STICK_DEADBAND_MIN   0.05f
#define FENCE_DIST_RELEASE_DELAY_US     500000      // roll and pitch centred this long hand the aircraft back
#define FENCE_DIST_ENGAGE_GRACE_US      500000      // position hold has this long to take the aircraft before it counts as failing

typedef enum {
    FENCE_RETURN_IDLE = 0,      // the pilot flies
    FENCE_RETURN_BRAKING,       // position hold brings the aircraft to a stop
    FENCE_RETURN_FLYING,        // flying back to the return point inside the radius
    FENCE_RETURN_HOLDING,       // holding inside the radius until the pilot centres the sticks
} fenceReturnState_e;

typedef struct fenceState_s {
    // ceiling
    bool altitudeValid;
    float altitudeCm;               // estimator frame, zeroed at arming
    float allowedClimbRateCmS;
    bool throttleCapActive;
    float throttleCap;
    float throttleCapI;
    bool throttleCapped;            // the mixer has applied the cap since the last update
    pt1Filter_t throttleCapLpf;     // the cap as the mixer applies it, at loop rate

    // throttle handover, at loop rate
    bool handoverOwned;             // fenceOwnsFlight() as the mixer last saw it
    float handoverFromThrottle;     // the throttle as it stood when the handover started
    float handoverProgress;         // 0..1 through the handover, 1 = done
    float handoverTimeS;
    float throttleOut;              // the throttle last handed to the mixer

    // radius
    bool distanceValid;
    float distanceCm;               // horizontal, from home
    float radialSpeedCmS;           // positive away from home
    vector2_t homeEfCm;             // home in the estimator frame
    fenceReturnState_e returnState;
    timeUs_t returnStateUs;
    timeUs_t engagedUs;             // when the fence last took the aircraft
    bool sticksCentred;
    timeUs_t sticksCentredUs;
    bool returnUnavailable;         // the radius is enforced but the fence cannot fly the aircraft
    bool returnLockout;             // a return was abandoned: none again until the aircraft is back inside
    bool ownsNavTarget;
    uint32_t navSequence;
} fenceState_t;

static fenceState_t fence;

const char * const fenceModeNames[FENCE_MODE_COUNT] = {
    "OFF", "WARN", "LIMIT"
};

void fenceInit(void)
{
    memset(&fence, 0, sizeof(fence));
    pt1FilterInit(&fence.throttleCapLpf, pt1FilterGain(FENCE_ALT_CAP_LPF_HZ, pidGetDT()));
    fence.throttleCapLpf.state = 1.0f;
    fence.handoverProgress = 1.0f;
}

// The fastest the aircraft may climb altitudeCm high and still stop at the ceiling, and
// above it, the (negative) rate it is pushed back down at.
STATIC_UNIT_TESTED float fenceAllowedClimbRateCmS(float altitudeCm, float ceilingCm)
{
    const float marginCm = ceilingCm - altitudeCm;
    if (marginCm <= 0.0f) {
        return fmaxf(marginCm * FENCE_ALT_POSITION_GAIN, -FENCE_ALT_RECOVERY_RATE_CMS);
    }
    // Braking onto the ceiling far from it, closing on it at the position gain near it,
    // where the square root would turn altitude noise into throttle noise.
    return fminf(sqrtf(2.0f * FENCE_ALT_BRAKE_DECEL_CMSS * marginCm), marginCm * FENCE_ALT_POSITION_GAIN);
}

// How far from home the aircraft would come to rest if position hold braked it now at decelCmSS
STATIC_UNIT_TESTED float fenceStopDistanceCm(float distanceCm, float radialSpeedCmS, float decelCmSS)
{
    const float outwardCmS = fmaxf(radialSpeedCmS, 0.0f);
    return distanceCm + sq(outwardCmS) / (2.0f * decelCmSS);
}

static float brakeAngleDeg(void)
{
    // bounded below too: a zero angle would plan on never stopping
    return constrain(MIN(fenceConfig()->brakeAngleDeg, autopilotConfig()->maxAngle), FENCE_BRAKE_ANGLE_MIN_DEG, FENCE_BRAKE_ANGLE_MAX_DEG);
}

// The deceleration braking at the brake angle is counted on for, cm/s^2
static float brakeDecelCmSS(void)
{
    return FENCE_DIST_BRAKE_EFFICIENCY * G_ACCELERATION * 100.0f * tan_approx(DEGREES_TO_RADIANS(brakeAngleDeg()));
}

static float returnRadiusCm(const fenceConfig_t *cfg)
{
    const float marginM = MAX(cfg->distanceWarningM, FENCE_DIST_RETURN_MARGIN_MIN_M);
    return fmaxf(cfg->maxDistanceM - marginM, cfg->maxDistanceM * 0.5f) * 100.0f;
}

static void updateCeiling(float dtS)
{
    const fenceConfig_t *cfg = fenceConfig();

    fence.altitudeValid = isAltitudeAvailable();
    fence.altitudeCm = getAltitudeCmControl();
    fence.allowedClimbRateCmS = fenceAllowedClimbRateCmS(fence.altitudeCm, cfg->maxAltitudeM * 100.0f);

    const bool enforced = ARMING_FLAG(ARMED)
        && cfg->altitudeMode == FENCE_MODE_LIMIT
        && fence.altitudeValid
        && !featureIsEnabled(FEATURE_3D);

    if (!enforced) {
        fence.throttleCapActive = false;
        fence.throttleCapI = 0.0f;
        fence.throttleCapped = false;
        return;
    }

    const float rateErrorCmS = fence.allowedClimbRateCmS - getAltitudeDerivativeControl();

    // The integral trims the hover estimate, so it learns only while the cap is what flies the aircraft
    if (fence.throttleCapped) {
        fence.throttleCapI += rateErrorCmS * FENCE_ALT_RATE_I_GAIN * dtS;
        fence.throttleCapI = constrainf(fence.throttleCapI, -FENCE_ALT_RATE_I_LIMIT, FENCE_ALT_RATE_I_LIMIT);
    } else {
        fence.throttleCapI -= fence.throttleCapI * fminf(FENCE_ALT_RATE_I_DECAY * dtS, 1.0f);
    }
    fence.throttleCapped = false;

    const float cosTilt = getCosTiltAngle();
    if (cosTilt <= 0.0f) {
        // Upside down, thrust pulls the aircraft away from the ceiling
        fence.throttleCapActive = false;
        return;
    }

    const float hoverThrottle = scaleRangef(autopilotGetEffectiveHoverThrottlePwm(), MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
    const float cap = (hoverThrottle + rateErrorCmS * FENCE_ALT_RATE_GAIN + fence.throttleCapI) / fmaxf(cosTilt, 0.5f);
    fence.throttleCap = constrainf(cap, 0.0f, 1.0f);
    fence.throttleCapActive = true;
}

static void updateDistance(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    gpsLocation_t origin;

    fence.distanceValid = STATE(GPS_FIX) && STATE(GPS_FIX_HOME) && est->isValidXY && positionEstimatorGetGpsOrigin(&origin);
    if (!fence.distanceValid) {
        return;
    }

    // The estimator frame starts where XY estimation did, which need not be home
    GPS_distance2d(&origin, &GPS_home_llh, &fence.homeEfCm);

    vector2_t fromHomeCm;
    vector2Sub(&fromHomeCm, (const vector2_t *)&est->position.v, &fence.homeEfCm);
    fence.distanceCm = vector2Norm(&fromHomeCm);
    fence.radialSpeedCmS = (fence.distanceCm > 1.0f)
        ? vector2Dot((const vector2_t *)&est->velocity.v, &fromHomeCm) / fence.distanceCm
        : 0.0f;
}

static void setReturnState(fenceReturnState_e state, timeUs_t currentTimeUs)
{
    fence.returnState = state;
    fence.returnStateUs = currentTimeUs;
    fence.sticksCentred = false;
}

static void releaseNavTarget(void)
{
    if (!fence.ownsNavTarget) {
        return;
    }
    // A mission or rescue that has since taken the navigator keeps its own target
    const positionNavCommand_t *cmd = positionNavGetActiveCommand();
    if (cmd->active && cmd->sequence == fence.navSequence) {
        positionNavClearTarget();
    }
    fence.ownsNavTarget = false;
}

// Straight back towards home, to the return point on the line from home to the aircraft.
// Altitude stays with alt hold, so the pilot keeps the throttle.
static void startReturnLeg(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();

    vector2_t returnPointCm;
    vector2Sub(&returnPointCm, (const vector2_t *)&est->position.v, &fence.homeEfCm);
    vector2Scale(&returnPointCm, &returnPointCm, returnRadiusCm(fenceConfig()) / fence.distanceCm);
    vector2Add(&returnPointCm, &returnPointCm, &fence.homeEfCm);

    const vector3_t targetM = {.v = {
        [ENU_E] = returnPointCm.x * 0.01f,
        [ENU_N] = returnPointCm.y * 0.01f,
        [ENU_U] = est->position.v[ENU_U] * 0.01f,
    }};
    positionNavSetTargetEf(&targetM, autopilotConfig()->maxVelocity * 0.01f, FENCE_DIST_ARRIVAL_RADIUS_M,
                           FENCE_DIST_ARRIVAL_SPEED_MPS, false, NULL, NULL);
    positionNavStartAfresh();
    positionNavSetSettleTimeout(FENCE_DIST_SETTLE_TIMEOUT_S);
    autopilotSetNavHeadingOverride(false, 0.0f);   // nose follows the way back

    fence.navSequence = positionNavGetActiveCommand()->sequence;
    fence.ownsNavTarget = true;
}

static bool sticksCentred(void)
{
    const float deadband = fmaxf(posHoldConfig()->deadband * 0.01f, FENCE_DIST_STICK_DEADBAND_MIN);
    return getRcDeflectionAbs(FD_ROLL) <= deadband && getRcDeflectionAbs(FD_PITCH) <= deadband;
}

static void updateReturn(timeUs_t currentTimeUs)
{
    const fenceConfig_t *cfg = fenceConfig();
    const float limitCm = cfg->maxDistanceM * 100.0f;

    const bool enforced = ARMING_FLAG(ARMED) && cfg->distanceMode == FENCE_MODE_LIMIT;
    // Whatever else has the aircraft has the better claim on it
    const bool deferred = failsafeIsActive()
        || FLIGHT_MODE(GPS_RESCUE_MODE | AUTOPILOT_MODE)
        || flightPlanNavIsRescueDescentActive();
    // Position hold flies the return, with alt hold under it
    const bool canFly = fence.distanceValid && fence.altitudeValid && posHoldReady();
    const bool allowed = enforced && !deferred && canFly;
    const bool breach = fence.distanceValid && fenceStopDistanceCm(fence.distanceCm, fence.radialSpeedCmS, brakeDecelCmSS()) >= limitCm;
    // Position hold has been given the aircraft but is not flying it (its own sanity checks tripped)
    const bool controlLost = fence.returnState != FENCE_RETURN_IDLE
        && cmpTimeUs(currentTimeUs, fence.engagedUs) > FENCE_DIST_ENGAGE_GRACE_US
        && !isAutopilotInControl();

    fence.returnUnavailable = enforced && !canFly;

    if (fence.returnState != FENCE_RETURN_IDLE && (!allowed || controlLost)) {
        // Lost the means to fly it back: hand it to the pilot, and do not snatch it again
        // the moment position hold looks ready, or the aircraft would be bounced between the two.
        fence.returnLockout = enforced && !deferred;
        releaseNavTarget();
        setReturnState(FENCE_RETURN_IDLE, currentTimeUs);
    }
    if (fence.returnLockout && (!ARMING_FLAG(ARMED) || (!breach && fence.distanceCm <= returnRadiusCm(cfg)))) {
        fence.returnLockout = false;
    }

    switch (fence.returnState) {
    case FENCE_RETURN_IDLE:
        if (allowed && breach && !fence.returnLockout) {
            fence.engagedUs = currentTimeUs;
            setReturnState(FENCE_RETURN_BRAKING, currentTimeUs);
        }
        break;

    case FENCE_RETURN_BRAKING:
        if (positionEstimateGroundspeedCmS(positionEstimatorGetEstimate()) < FENCE_DIST_STOPPED_SPEED_CMS
            || cmpTimeUs(currentTimeUs, fence.returnStateUs) > FENCE_DIST_BRAKING_TIMEOUT_US) {
            if (fence.distanceCm > returnRadiusCm(cfg)) {
                startReturnLeg();
                setReturnState(FENCE_RETURN_FLYING, currentTimeUs);
            } else {
                // Braking alone stopped it short of the return point
                setReturnState(FENCE_RETURN_HOLDING, currentTimeUs);
            }
        }
        break;

    case FENCE_RETURN_FLYING:
        if (!positionNavHasActiveTarget() || positionNavTargetReached()) {
            releaseNavTarget();
            setReturnState(FENCE_RETURN_HOLDING, currentTimeUs);
        } else if (cmpTimeUs(currentTimeUs, fence.returnStateUs) > FENCE_DIST_RETURN_TIMEOUT_US) {
            // Never got there: rather than hold the pilot off indefinitely, hand the aircraft back
            fence.returnLockout = true;
            releaseNavTarget();
            setReturnState(FENCE_RETURN_IDLE, currentTimeUs);
        }
        break;

    case FENCE_RETURN_HOLDING:
        // The pilot flies position hold from here, and may fly it straight back out
        if (breach) {
            setReturnState(FENCE_RETURN_BRAKING, currentTimeUs);
        } else if (!sticksCentred()) {
            fence.sticksCentred = false;
        } else if (!fence.sticksCentred) {
            fence.sticksCentred = true;
            fence.sticksCentredUs = currentTimeUs;
        } else if (cmpTimeUs(currentTimeUs, fence.sticksCentredUs) >= FENCE_DIST_RELEASE_DELAY_US) {
            setReturnState(FENCE_RETURN_IDLE, currentTimeUs);
        }
        break;
    }
}

void fenceUpdate(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUs;
    const float dtS = constrainf(US_TO_INTERVAL(cmpTimeUs(currentTimeUs, previousTimeUs)), 0.0f, 0.1f);
    previousTimeUs = currentTimeUs;

    updateCeiling(dtS);
    updateDistance();
    updateReturn(currentTimeUs);

    DEBUG_SET(DEBUG_FENCE, 0, fence.returnState);                                   //!< Return State [enum:fenceReturnState_e]
    DEBUG_SET(DEBUG_FENCE, 1, lrintf(fence.distanceCm * 0.01f));                    //!< Distance From Home [unit:m]
    DEBUG_SET(DEBUG_FENCE, 2, lrintf(fence.radialSpeedCmS));                        //!< Speed Away From Home [unit:cm/s]
    DEBUG_SET(DEBUG_FENCE, 3, lrintf(fenceStopDistanceCm(fence.distanceCm, fence.radialSpeedCmS, brakeDecelCmSS()) * 0.01f)); //!< Stopping Distance From Home [unit:m]
    DEBUG_SET(DEBUG_FENCE, 4, lrintf(fence.altitudeCm * 0.1f));                     //!< Altitude [unit:0.1m]
    DEBUG_SET(DEBUG_FENCE, 5, lrintf(fence.allowedClimbRateCmS));                   //!< Allowed Climb Rate [unit:cm/s]
    DEBUG_SET(DEBUG_FENCE, 6, lrintf((fence.throttleCapActive ? fence.throttleCap : 1.0f) * 1000.0f)); //!< Throttle Cap [unit:0.001]
    DEBUG_SET(DEBUG_FENCE, 7, lrintf(fence.throttleCapI * 1000.0f));                //!< Throttle Cap Integral [unit:0.001]
}

bool fenceIsReadyToArm(void)
{
    const fenceConfig_t *cfg = fenceConfig();

    if (cfg->altitudeMode == FENCE_MODE_LIMIT && !isAltitudeAvailable()) {
        return false;
    }
    // Home is taken with this many satellites (GPS_reset_home_position) and the radius is measured from it
    if (cfg->distanceMode == FENCE_MODE_LIMIT && !(STATE(GPS_FIX) && gpsSol.numSat >= gpsRescueConfig()->minSats)) {
        return false;
    }
    return true;
}

float fenceHandoverThrottle(float throttle)
{
    if (!ARMING_FLAG(ARMED)) {
        fence.handoverOwned = false;
        fence.handoverProgress = 1.0f;
        fence.throttleOut = throttle;
        return throttle;
    }

    const bool owned = fenceOwnsFlight();
    if (owned != fence.handoverOwned) {
        // From wherever the throttle stands, so a handover cut short by another starts from where it got to
        fence.handoverOwned = owned;
        fence.handoverFromThrottle = fence.throttleOut;
        fence.handoverProgress = 0.0f;
        fence.handoverTimeS = owned ? FENCE_TAKEOVER_TIME_S : FENCE_HANDBACK_TIME_S;
    }
    if (fence.handoverProgress < 1.0f) {
        fence.handoverProgress = fminf(fence.handoverProgress + pidGetDT() / fence.handoverTimeS, 1.0f);
        throttle = fence.handoverFromThrottle + (throttle - fence.handoverFromThrottle) * fence.handoverProgress;
    }
    fence.throttleOut = throttle;
    return throttle;
}

float fenceLimitThrottle(float throttle)
{
    if (!fence.throttleCapActive) {
        // The cap eases in from no cap, and lets go at once
        fence.throttleCapLpf.state = 1.0f;
        return throttle;
    }
    const float cap = pt1FilterApply(&fence.throttleCapLpf, fence.throttleCap);
    if (throttle > cap) {
        fence.throttleCapped = true;
        return cap;
    }
    return throttle;
}

bool fenceConstrainAltitudeTarget(float *targetAltitudeCm)
{
    const fenceConfig_t *cfg = fenceConfig();
    if (cfg->altitudeMode != FENCE_MODE_LIMIT) {
        return false;
    }
    const float maxTargetCm = cfg->maxAltitudeM * 100.0f - FENCE_ALT_TARGET_MARGIN_CM;
    if (*targetAltitudeCm > maxTargetCm) {
        *targetAltitudeCm = maxTargetCm;
        return true;
    }
    return false;
}

bool fenceOwnsFlight(void)
{
    return fence.returnState != FENCE_RETURN_IDLE;
}

bool fenceOwnsPosition(void)
{
    return fence.returnState == FENCE_RETURN_BRAKING || fence.returnState == FENCE_RETURN_FLYING;
}

float fenceGetMaxAngleDeg(void)
{
    return fenceOwnsPosition() ? brakeAngleDeg() : 0.0f;
}

uint16_t fenceGetDistanceLimitM(void)
{
    return fenceConfig()->distanceMode == FENCE_MODE_LIMIT ? fenceConfig()->maxDistanceM : 0;
}

int fenceGetAltitudeMarginM(void)
{
    return lrintf(fenceConfig()->maxAltitudeM - fence.altitudeCm * 0.01f);
}

int fenceGetDistanceMarginM(void)
{
    return lrintf(fenceConfig()->maxDistanceM - fence.distanceCm * 0.01f);
}

fenceWarning_e fenceGetWarning(void)
{
    if (!ARMING_FLAG(ARMED)) {
        return FENCE_WARNING_NONE;
    }

    switch (fence.returnState) {
    case FENCE_RETURN_BRAKING:
    case FENCE_RETURN_FLYING:
        return FENCE_WARNING_RETURNING;
    case FENCE_RETURN_HOLDING:
        return FENCE_WARNING_HOLDING;
    default:
        break;
    }

    const fenceConfig_t *cfg = fenceConfig();
    const bool altitudeChecked = cfg->altitudeMode != FENCE_MODE_OFF && fence.altitudeValid;
    const bool distanceChecked = cfg->distanceMode != FENCE_MODE_OFF && fence.distanceValid;

    if (altitudeChecked && fenceGetAltitudeMarginM() <= 0) {
        return FENCE_WARNING_ALTITUDE_MAX;
    }
    if (distanceChecked && fenceGetDistanceMarginM() <= 0) {
        return FENCE_WARNING_DISTANCE_MAX;
    }
    if ((cfg->altitudeMode != FENCE_MODE_OFF && !fence.altitudeValid)
        || (cfg->distanceMode != FENCE_MODE_OFF && !fence.distanceValid)
        || fence.returnUnavailable) {
        return FENCE_WARNING_UNAVAILABLE;
    }
    if (altitudeChecked && fenceGetAltitudeMarginM() <= cfg->altitudeWarningM) {
        return FENCE_WARNING_ALTITUDE_NEAR;
    }
    if (distanceChecked && fenceGetDistanceMarginM() <= cfg->distanceWarningM) {
        return FENCE_WARNING_DISTANCE_NEAR;
    }
    return FENCE_WARNING_NONE;
}

#endif // USE_FENCE
