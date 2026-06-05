#include "drum_pipeline.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// convert radians to degrees
static float deg(float rad) { return rad * 180.0f / M_PI; }

// look back n samples in ring buffer (0 = most recent)
static const dp_sample_t *ring_ago(const dp_state_t *s, int n)
{
    if (n >= s->ring_count) n = s->ring_count - 1;
    if (n < 0) return NULL;
    int idx = (s->ring_head - 1 - n + DP_RING_SIZE) % DP_RING_SIZE;
    return &s->ring[idx];
}

void dp_init(dp_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->initialized = false;
}

bool dp_update(dp_state_t *s,
               int16_t raw_ax, int16_t raw_ay, int16_t raw_az,
               int16_t raw_gx, int16_t raw_gy, int16_t raw_gz,
               int64_t now_us,
               dp_hit_event_t *hit_out)
{
    // convert to real units
    float ax = (float)raw_ax / DP_ACCEL_SCALE;
    float ay = (float)raw_ay / DP_ACCEL_SCALE;
    float az = (float)raw_az / DP_ACCEL_SCALE;
    float gx = (float)raw_gx / DP_GYRO_SCALE;
    float gy = (float)raw_gy / DP_GYRO_SCALE;
    float gz = (float)raw_gz / DP_GYRO_SCALE;

    float accel_mag = sqrtf(ax*ax + ay*ay + az*az);

    // first sample - just set initial angles from accel
    if (!s->initialized) {
        s->pitch = deg(atan2f(ay, az));
        s->roll  = deg(atan2f(ax, sqrtf(ay*ay + az*az)));
        s->prev_accel_mag = accel_mag;
        s->last_hit_us    = now_us - (int64_t)DP_HIT_COOLDOWN_MS * 1000;
        s->freeze_until_us = 0;
        s->initialized    = true;
    }

    // figure out dt from last sample
    float dt = 0.01f;
    if (s->ring_count > 0) {
        const dp_sample_t *prev = ring_ago(s, 0);
        if (prev && now_us > prev->timestamp_us) {
            dt = (float)(now_us - prev->timestamp_us) / 1e6f;
            if (dt > 0.1f) dt = 0.1f;
        }
    }

    // accel-based angles (only good when not moving fast)
    // roll uses sqrt(ay²+az²) as denominator — stable at steep pitch angles where az alone is tiny
    float accel_pitch = deg(atan2f(ay, az));
    float accel_roll  = deg(atan2f(ax, sqrtf(ay*ay + az*az)));

    // adaptive trust: accel_roll is contaminated by any linear acceleration along az
    // (e.g. moving the arm up/down) even when accel_mag stays near 1g.
    // Solution: trust gyro exclusively during motion; blend accel only when stationary.
    float gyro_total = sqrtf(gx*gx + gy*gy + gz*gz);
    bool  noisy_accel = fabsf(accel_mag - 1.0f) > DP_ACCEL_TRUST_G;
    bool  moving      = gyro_total > DP_MOVING_GYRO_DPS;
    bool  frozen      = now_us < s->freeze_until_us;

    // when the arm is clearly still, learn the gyro Y-axis bias to cancel long-term drift
    if (!moving && !noisy_accel && !frozen) {
        s->gyro_bias_y = 0.995f * s->gyro_bias_y + 0.005f * gy;
    }

    float alpha = DP_ALPHA;
    float alpha_roll;
    if (noisy_accel || frozen) {
        alpha      = 1.0f;
        alpha_roll = 1.0f;     // definitely mid-swing or vibrating
    } else if (moving) {
        alpha_roll = DP_ROLL_ALPHA_MOVE;   // arm in motion — gyro only
    } else {
        alpha_roll = DP_ROLL_ALPHA_STILL;  // genuinely still — let accel correct drift
    }

    // complementary filter — subtract learned bias from gy before integrating roll
    float gyro_pitch = s->pitch + gx * dt;
    float gyro_roll  = s->roll  + (gy - s->gyro_bias_y) * dt;

    s->pitch = alpha      * gyro_pitch + (1.0f - alpha)      * accel_pitch;
    s->roll  = alpha_roll * gyro_roll  + (1.0f - alpha_roll) * accel_roll;

    // store in ring buffer
    dp_sample_t *slot = &s->ring[s->ring_head];
    slot->timestamp_us = now_us;
    slot->ax = ax;  slot->ay = ay;  slot->az = az;
    slot->gx = gx;  slot->gy = gy;  slot->gz = gz;
    slot->pitch    = s->pitch;
    slot->roll     = s->roll;
    slot->accel_mag = accel_mag;
    s->ring_head = (s->ring_head + 1) % DP_RING_SIZE;
    if (s->ring_count < DP_RING_SIZE) s->ring_count++;

    // hit detection: threshold + rising edge + downswing direction + cooldown
    const dp_sample_t *pre1 = ring_ago(s, 1);
    bool was_downswing = pre1 && (pre1->gx > DP_DOWNSWING_GX_MIN);

    bool hit = false;
    if (accel_mag >= DP_HIT_THRESHOLD_G &&
        accel_mag > s->prev_accel_mag &&
        was_downswing &&
        (now_us - s->last_hit_us) >= (int64_t)DP_HIT_COOLDOWN_MS * 1000) {

        hit = true;
        s->last_hit_us     = now_us;
        s->freeze_until_us = now_us + (int64_t)DP_IMPACT_FREEZE_MS * 1000;

        const dp_sample_t *snap_now = ring_ago(s, 0);
        const dp_sample_t *snap_pre = ring_ago(s, DP_PRE_HIT_SAMPLES);

        if (hit_out && snap_now) {
            hit_out->timestamp_us    = snap_now->timestamp_us;
            hit_out->pitch           = snap_now->pitch;
            hit_out->roll            = snap_now->roll;
            hit_out->gyro_pitch_rate = snap_now->gx;
            hit_out->gyro_roll_rate  = snap_now->gy;
            hit_out->accel_peak_g    = snap_now->accel_mag;
            // ax/ay from the pre-hit sample — arm at rest position, no inertial contamination
            hit_out->ax = snap_pre ? snap_pre->ax : snap_now->ax;
            hit_out->ay = snap_pre ? snap_pre->ay : snap_now->ay;

            if (snap_pre) {
                hit_out->pre_pitch = snap_pre->pitch;
                hit_out->pre_roll  = snap_pre->roll;
            } else {
                hit_out->pre_pitch = hit_out->pitch;
                hit_out->pre_roll  = hit_out->roll;
            }

            float peak = hit_out->accel_peak_g;
            for (int i = 1; i < s->ring_count && i < DP_PRE_HIT_SAMPLES + 2; i++) {
                const dp_sample_t *p = ring_ago(s, i);
                if (p && p->accel_mag > peak) peak = p->accel_mag;
            }
            hit_out->accel_peak_g = peak;
        }

        // snap: classify using fresh accel-based roll (no gyro drift)
        float snap_ax = snap_pre ? snap_pre->ax : (snap_now ? snap_now->ax : 0.0f);
        float snap_ay = snap_pre ? snap_pre->ay : (snap_now ? snap_now->ay : 0.0f);
        float snap_az = snap_pre ? snap_pre->az : (snap_now ? snap_now->az : 1.0f);
        float fresh_roll = deg(atan2f(snap_ax, sqrtf(snap_ay*snap_ay + snap_az*snap_az)));

        s->pitch = (s->pitch   < DP_ZONE_PITCH_THRESHOLD) ? DP_ANCHOR_UP_PITCH  : DP_ANCHOR_DOWN_PITCH;
        s->roll  = (fresh_roll < DP_ZONE_ROLL_THRESHOLD)   ? DP_ANCHOR_LEFT_ROLL : DP_ANCHOR_RIGHT_ROLL;
    }

    s->prev_accel_mag = accel_mag;
    return hit;
}

void dp_get_orientation(const dp_state_t *s, float *pitch, float *roll)
{
    *pitch = s->pitch;
    *roll  = s->roll;
}
