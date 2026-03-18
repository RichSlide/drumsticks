#include "drum_pipeline.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ========================================================================
 *  HELPERS
 * ======================================================================== */

static inline float deg(float rad) { return rad * (180.0f / (float)M_PI); }

/* Single-pole low-pass: out = alpha * prev + (1-alpha) * new */
static inline float lpf(float prev, float new_val, float alpha)
{
    return alpha * prev + (1.0f - alpha) * new_val;
}

/* Look back `n` samples in the ring buffer.  n=0 is the most recent. */
static const dp_sample_t *ring_ago(const dp_state_t *s, int n)
{
    if (n >= s->ring_count) n = s->ring_count - 1;
    if (n < 0) return NULL;
    int idx = (s->ring_head - 1 - n + DP_RING_SIZE) % DP_RING_SIZE;
    return &s->ring[idx];
}

/* ========================================================================
 *  INIT
 * ======================================================================== */

void dp_init(dp_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->initialized = false;
}

/* ========================================================================
 *  COMPUTE ACCELEROMETER-DERIVED ANGLES
 *
 *  These are only valid when the stick is roughly stationary (accel ≈ gravity).
 *  During fast motion or impact the accel vector contains centripetal and
 *  impact forces, so these angles are garbage — that's why we gate them.
 * ======================================================================== */

static void compute_accel_angles(float ax, float ay, float az,
                                 float *pitch_out, float *roll_out)
{
    *pitch_out = deg(atan2f(ay, az));
    *roll_out  = deg(atan2f(ax, az));
}

/* ========================================================================
 *  UPDATE ORIENTATION — complementary filter with accel gating
 *
 *  Core idea:
 *    1. Integrate gyro for fast, drift-prone angle update.
 *    2. Slowly blend toward accelerometer-derived angle to kill drift.
 *    3. BUT only blend accel when accel magnitude is near 1g, meaning
 *       the stick is not under dynamic acceleration (swing / impact).
 *    4. After a detected hit, freeze accel correction entirely for a
 *       short window so vibration doesn't corrupt orientation.
 * ======================================================================== */

static void update_orientation(dp_state_t *s,
                               float gx_dps, float gy_dps,
                               float accel_pitch, float accel_roll,
                               float accel_mag_g,
                               float dt,
                               int64_t now_us)
{
    /* Gyro integration */
    float gyro_pitch = s->pitch + gx_dps * dt;
    float gyro_roll  = s->roll  + gy_dps * dt;

    /* Decide how much to trust the accelerometer this sample */
    float alpha = DP_ALPHA;

    /* Gate 1: accel magnitude far from 1g → dynamic motion, don't trust */
    if (fabsf(accel_mag_g - 1.0f) > DP_ACCEL_TRUST_G) {
        alpha = 1.0f;  /* pure gyro */
    }

    /* Gate 2: inside post-impact freeze window → pure gyro */
    if (now_us < s->freeze_until_us) {
        alpha = 1.0f;
    }

    /* Complementary blend */
    s->pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
    s->roll  = alpha * gyro_roll  + (1.0f - alpha) * accel_roll;
}

/* ========================================================================
 *  HIT DETECTION
 * ======================================================================== */

static bool detect_hit(dp_state_t *s, float accel_mag_g, float jerk,
                       int64_t now_us)
{
    /* Cooldown check */
    if ((now_us - s->last_hit_us) < (int64_t)DP_HIT_COOLDOWN_MS * 1000) {
        return false;
    }

    /* Magnitude threshold */
    if (accel_mag_g < DP_HIT_THRESHOLD_G) {
        return false;
    }

    /* Optional jerk threshold */
    if (DP_JERK_THRESHOLD_G > 0.0f && jerk < DP_JERK_THRESHOLD_G) {
        return false;
    }

    return true;
}

/* ========================================================================
 *  FEATURE EXTRACTION
 *
 *  Pulls orientation, velocity, and accel features from the ring buffer
 *  at the moment of impact and from a short pre-hit window.
 * ======================================================================== */

static void extract_hit_features(const dp_state_t *s, float jerk,
                                 dp_hit_event_t *out)
{
    const dp_sample_t *now = ring_ago(s, 0);
    if (!now) return;

    out->timestamp_us    = now->timestamp_us;
    out->pitch           = now->pitch;
    out->roll            = now->roll;
    out->gyro_pitch_rate = now->gx;
    out->gyro_roll_rate  = now->gy;
    out->accel_peak_g    = now->accel_mag;
    out->jerk            = jerk;
    out->ax              = now->ax;
    out->ay              = now->ay;

    /* Pre-hit orientation (a few samples earlier) */
    const dp_sample_t *pre = ring_ago(s, DP_PRE_HIT_SAMPLES);
    if (pre) {
        out->pre_pitch = pre->pitch;
        out->pre_roll  = pre->roll;
    } else {
        out->pre_pitch = out->pitch;
        out->pre_roll  = out->roll;
    }

    /* Scan ring for peak accel near hit */
    float peak = out->accel_peak_g;
    for (int i = 1; i < s->ring_count && i < DP_PRE_HIT_SAMPLES + 2; i++) {
        const dp_sample_t *p = ring_ago(s, i);
        if (p && p->accel_mag > peak) peak = p->accel_mag;
    }
    out->accel_peak_g = peak;
}

/* ========================================================================
 *  MAIN UPDATE — call once per IMU sample
 * ======================================================================== */

bool dp_update(dp_state_t *s,
               int16_t raw_ax, int16_t raw_ay, int16_t raw_az,
               int16_t raw_gx, int16_t raw_gy, int16_t raw_gz,
               int64_t now_us,
               dp_hit_event_t *hit_out)
{
    /* Convert to physical units */
    float ax = (float)raw_ax / DP_ACCEL_SCALE;   /* g    */
    float ay = (float)raw_ay / DP_ACCEL_SCALE;
    float az = (float)raw_az / DP_ACCEL_SCALE;
    float gx = (float)raw_gx / DP_GYRO_SCALE;    /* deg/s */
    float gy = (float)raw_gy / DP_GYRO_SCALE;
    float gz = (float)raw_gz / DP_GYRO_SCALE;

    float accel_mag = sqrtf(ax*ax + ay*ay + az*az);

    /* Bootstrap on first sample */
    if (!s->initialized) {
        compute_accel_angles(ax, ay, az, &s->pitch, &s->roll);
        s->accel_mag_lpf  = accel_mag;
        s->prev_accel_mag = accel_mag;
        s->last_hit_us    = now_us - (int64_t)DP_HIT_COOLDOWN_MS * 1000;
        s->freeze_until_us = 0;
        s->initialized    = true;
    }

    /* Compute dt from the previous sample in the ring */
    float dt = 0.01f;  /* fallback: 10 ms */
    if (s->ring_count > 0) {
        const dp_sample_t *prev = ring_ago(s, 0);
        if (prev && now_us > prev->timestamp_us) {
            dt = (float)(now_us - prev->timestamp_us) / 1e6f;
            if (dt > 0.1f) dt = 0.1f;  /* clamp absurd gaps */
        }
    }

    /* Accelerometer-derived reference angles */
    float accel_pitch, accel_roll;
    compute_accel_angles(ax, ay, az, &accel_pitch, &accel_roll);

    /* Update orientation with gated complementary filter */
    update_orientation(s, gx, gy, accel_pitch, accel_roll, accel_mag, dt, now_us);

    /* Low-pass filter accel magnitude for smoother hit detection */
    s->accel_mag_lpf = lpf(s->accel_mag_lpf, accel_mag, DP_ACCEL_LPF_ALPHA);
    float jerk = s->accel_mag_lpf - s->prev_accel_mag;

    /* Store sample in ring buffer (after orientation is computed) */
    dp_sample_t *slot = &s->ring[s->ring_head];
    slot->timestamp_us = now_us;
    slot->ax = ax;  slot->ay = ay;  slot->az = az;
    slot->gx = gx;  slot->gy = gy;  slot->gz = gz;
    slot->pitch = s->pitch;
    slot->roll  = s->roll;
    slot->accel_mag = accel_mag;
    s->ring_head = (s->ring_head + 1) % DP_RING_SIZE;
    if (s->ring_count < DP_RING_SIZE) s->ring_count++;

    /* Hit detection */
    bool hit = detect_hit(s, s->accel_mag_lpf, jerk, now_us);

    if (hit) {
        s->last_hit_us = now_us;
        /* Freeze accel correction so impact vibration doesn't corrupt angles */
        s->freeze_until_us = now_us + (int64_t)DP_IMPACT_FREEZE_MS * 1000;

        if (hit_out) {
            extract_hit_features(s, jerk, hit_out);
        }
    }

    s->prev_accel_mag = s->accel_mag_lpf;

    return hit;
}

/* ========================================================================
 *  GETTERS
 * ======================================================================== */

void dp_get_orientation(const dp_state_t *s, float *pitch, float *roll)
{
    *pitch = s->pitch;
    *roll  = s->roll;
}

/* ========================================================================
 *  DEBUG OUTPUT
 *
 *  Format: P:<pitch> R:<roll> A:<accel_mag> G:<gyro_mag> HIT:<0|1>
 * ======================================================================== */

int dp_debug_line(const dp_state_t *s, const dp_sample_t *latest,
                  bool hit, char *buf, int buf_len)
{
    float gyro_mag = sqrtf(latest->gx * latest->gx +
                           latest->gy * latest->gy +
                           latest->gz * latest->gz);

    return snprintf(buf, buf_len,
                    "P:%+7.1f R:%+7.1f A:%5.2f G:%7.1f HIT:%d",
                    s->pitch, s->roll, latest->accel_mag, gyro_mag, hit ? 1 : 0);
}
