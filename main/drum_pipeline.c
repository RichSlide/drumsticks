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

    // first sample — pitch from accel; heading starts at 0 (current direction = neutral)
    if (!s->initialized) {
        s->pitch = deg(atan2f(ay, az));
        s->roll  = 0.0f;
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

    // pitch: complementary filter — accel gives absolute reference, gyro integrates between
    float accel_pitch = deg(atan2f(ay, az));
    float gyro_total  = sqrtf(gx*gx + gy*gy + gz*gz);
    bool  noisy_accel = fabsf(accel_mag - 1.0f) > DP_ACCEL_TRUST_G;
    bool  moving      = gyro_total > DP_MOVING_GYRO_DPS;
    bool  frozen      = now_us < s->freeze_until_us;

    float alpha = (noisy_accel || frozen) ? 1.0f : DP_ALPHA;
    s->pitch = alpha * (s->pitch + gx * dt) + (1.0f - alpha) * accel_pitch;

    // heading (stored in roll field): pure gz integration — no accel reference for yaw.
    // learn gz DC bias when the stick is stationary so long-term drift cancels out.
    if (!moving && !noisy_accel && !frozen) {
        s->gyro_bias_z = 0.995f * s->gyro_bias_z + 0.005f * gz;
    }
    s->roll = s->roll - (gz - s->gyro_bias_z) * dt;

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
    // check last 3 samples — immune to FreeRTOS scheduler jitter shifting the window
    bool was_downswing = false;
    for (int i = 1; i <= 3 && !was_downswing; i++) {
        const dp_sample_t *p = ring_ago(s, i);
        if (p) was_downswing = (p->gx > DP_DOWNSWING_GX_MIN);
    }

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
            hit_out->gyro_roll_rate  = snap_now->gz;
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

    }

    s->prev_accel_mag = accel_mag;
    return hit;
}

void dp_get_orientation(const dp_state_t *s, float *pitch, float *roll)
{
    *pitch = s->pitch;
    *roll  = s->roll;
}
