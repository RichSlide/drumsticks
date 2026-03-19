#ifndef DRUM_PIPELINE_H
#define DRUM_PIPELINE_H

#include <stdint.h>
#include <stdbool.h>

// IMU conversion (ICM-20948 defaults: +-250 dps, +-2g)
#define DP_GYRO_SCALE       131.0f
#define DP_ACCEL_SCALE      16384.0f

// complementary filter weight (higher = trust gyro more)
#define DP_ALPHA            0.98f

// dont use accel for angle if magnitude is too far from 1g
#define DP_ACCEL_TRUST_G    0.3f

// freeze accel correction after a hit so vibrations dont mess up angles
#define DP_IMPACT_FREEZE_MS 40

// hit threshold in g
#define DP_HIT_THRESHOLD_G  2.0f

// debounce between hits (ms)
#define DP_HIT_COOLDOWN_MS  80

// low pass filter for accel magnitude
#define DP_ACCEL_LPF_ALPHA  0.3f

// ring buffer for storing recent samples
#define DP_RING_SIZE        16
#define DP_PRE_HIT_SAMPLES  3

// one IMU sample with orientation
typedef struct {
    int64_t  timestamp_us;
    float    ax, ay, az;
    float    gx, gy, gz;
    float    pitch, roll;
    float    accel_mag;
} dp_sample_t;

// info about a detected hit
typedef struct {
    int64_t  timestamp_us;
    float    pitch, roll;
    float    pre_pitch, pre_roll;
    float    gyro_pitch_rate, gyro_roll_rate;
    float    accel_peak_g;
    float    ax, ay;
} dp_hit_event_t;

// pipeline state
typedef struct {
    float    pitch, roll;
    float    accel_mag_lpf;
    float    prev_accel_mag;
    int64_t  last_hit_us;
    int64_t  freeze_until_us;
    bool     initialized;
    dp_sample_t ring[DP_RING_SIZE];
    int      ring_head;
    int      ring_count;
} dp_state_t;

void dp_init(dp_state_t *s);

bool dp_update(dp_state_t *s,
               int16_t raw_ax, int16_t raw_ay, int16_t raw_az,
               int16_t raw_gx, int16_t raw_gy, int16_t raw_gz,
               int64_t now_us,
               dp_hit_event_t *hit_out);

void dp_get_orientation(const dp_state_t *s, float *pitch, float *roll);

#endif
