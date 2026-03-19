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
        s->roll  = deg(atan2f(ax, az));
        s->accel_mag_lpf  = accel_mag;
        s->prev_accel_mag = accel_mag;
        s->last_hit_us    = now_us - (int64_t)DP_HIT_COOLDOWN_MS * 1000;
        s->freeze_until_us = 0;
        s->initialized = true;
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
    float accel_pitch = deg(atan2f(ay, az));
    float accel_roll  = deg(atan2f(ax, az));

    // complementary filter - integrate gyro then blend with accel
    float gyro_pitch = s->pitch + gx * dt;
    float gyro_roll  = s->roll  + gy * dt;

    float alpha = DP_ALPHA;

    // if accel is far from 1g, stick is moving fast - dont trust accel
    if (fabsf(accel_mag - 1.0f) > DP_ACCEL_TRUST_G)
        alpha = 1.0f;

    // after a hit, ignore accel for a bit so vibrations dont mess things up
    if (now_us < s->freeze_until_us)
        alpha = 1.0f;

    s->pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
    s->roll  = alpha * gyro_roll  + (1.0f - alpha) * accel_roll;

    // low pass filter on accel magnitude for hit detection
    s->accel_mag_lpf = DP_ACCEL_LPF_ALPHA * s->accel_mag_lpf +
                       (1.0f - DP_ACCEL_LPF_ALPHA) * accel_mag;

    // store in ring buffer
    dp_sample_t *slot = &s->ring[s->ring_head];
    slot->timestamp_us = now_us;
    slot->ax = ax;  slot->ay = ay;  slot->az = az;
    slot->gx = gx;  slot->gy = gy;  slot->gz = gz;
    slot->pitch = s->pitch;
    slot->roll  = s->roll;
    slot->accel_mag = accel_mag;
    s->ring_head = (s->ring_head + 1) % DP_RING_SIZE;
    if (s->ring_count < DP_RING_SIZE) s->ring_count++;

    // check for hit
    bool hit = false;
    if (s->accel_mag_lpf >= DP_HIT_THRESHOLD_G &&
        (now_us - s->last_hit_us) >= (int64_t)DP_HIT_COOLDOWN_MS * 1000) {

        hit = true;
        s->last_hit_us = now_us;
        s->freeze_until_us = now_us + (int64_t)DP_IMPACT_FREEZE_MS * 1000;

        if (hit_out) {
            const dp_sample_t *now = ring_ago(s, 0);
            if (now) {
                hit_out->timestamp_us    = now->timestamp_us;
                hit_out->pitch           = now->pitch;
                hit_out->roll            = now->roll;
                hit_out->gyro_pitch_rate = now->gx;
                hit_out->gyro_roll_rate  = now->gy;
                hit_out->accel_peak_g    = now->accel_mag;
                hit_out->ax              = now->ax;
                hit_out->ay              = now->ay;

                // grab orientation from a few samples ago
                const dp_sample_t *pre = ring_ago(s, DP_PRE_HIT_SAMPLES);
                if (pre) {
                    hit_out->pre_pitch = pre->pitch;
                    hit_out->pre_roll  = pre->roll;
                } else {
                    hit_out->pre_pitch = hit_out->pitch;
                    hit_out->pre_roll  = hit_out->roll;
                }

                // find peak accel near the hit
                float peak = hit_out->accel_peak_g;
                for (int i = 1; i < s->ring_count && i < DP_PRE_HIT_SAMPLES + 2; i++) {
                    const dp_sample_t *p = ring_ago(s, i);
                    if (p && p->accel_mag > peak) peak = p->accel_mag;
                }
                hit_out->accel_peak_g = peak;
            }
        }
    }

    s->prev_accel_mag = s->accel_mag_lpf;
    return hit;
}

void dp_get_orientation(const dp_state_t *s, float *pitch, float *roll)
{
    *pitch = s->pitch;
    *roll  = s->roll;
}
