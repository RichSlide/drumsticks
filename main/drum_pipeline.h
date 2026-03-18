#ifndef DRUM_PIPELINE_H
#define DRUM_PIPELINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================
 *  TUNABLE CONSTANTS — adjust these for your stick / playing style
 * ======================================================================== */

/* IMU conversion factors (ICM-20948 defaults: ±250 dps, ±2g) */
#define DP_GYRO_SCALE       131.0f    /* LSB per deg/s  */
#define DP_ACCEL_SCALE      16384.0f  /* LSB per g      */

/* Complementary filter: higher alpha = trust gyro more */
#define DP_ALPHA            0.98f

/* Accelerometer gating: only trust accel angle when magnitude is near 1g.
 * Units: g.  If |accel_mag - 1.0| > this, accel correction is suppressed. */
#define DP_ACCEL_TRUST_G    0.3f

/* After a hit, freeze accel correction for this many milliseconds
 * so impact vibration can't corrupt the orientation estimate. */
#define DP_IMPACT_FREEZE_MS 40

/* Hit detection: accel magnitude must exceed this (in g) */
#define DP_HIT_THRESHOLD_G  2.0f

/* Optional jerk threshold (delta-accel-mag per sample, in g).
 * Set to 0 to disable jerk gating. */
#define DP_JERK_THRESHOLD_G 0.0f

/* Minimum time between two hits (ms) — debounce / refractory */
#define DP_HIT_COOLDOWN_MS  80

/* Simple single-pole low-pass for accel magnitude used in hit detection.
 * 0 = raw, 1 = fully smoothed.  ~0.3 is a light filter. */
#define DP_ACCEL_LPF_ALPHA  0.3f

/* Ring buffer depth — stores recent samples for pre-hit feature extraction.
 * At 100 Hz, 16 samples ≈ 160 ms of history. */
#define DP_RING_SIZE        16

/* How many samples before the hit to use for "pre-hit" features */
#define DP_PRE_HIT_SAMPLES  3

/* ========================================================================
 *  DATA STRUCTURES
 * ======================================================================== */

/* One timestamped IMU sample with derived quantities */
typedef struct {
    int64_t  timestamp_us;      /* microseconds (esp_timer_get_time) */
    float    ax, ay, az;        /* accelerometer in g               */
    float    gx, gy, gz;        /* gyroscope in deg/s               */
    float    pitch, roll;       /* orientation estimate (degrees)    */
    float    accel_mag;         /* sqrt(ax²+ay²+az²)                */
} dp_sample_t;

/* Features extracted around each detected hit */
typedef struct {
    int64_t  timestamp_us;      /* when the hit was detected           */
    float    pitch;             /* orientation at impact               */
    float    roll;
    float    pre_pitch;         /* orientation ~pre-hit window before  */
    float    pre_roll;
    float    gyro_pitch_rate;   /* angular velocity at impact (deg/s)  */
    float    gyro_roll_rate;
    float    accel_peak_g;      /* peak accel magnitude near hit       */
    float    jerk;              /* accel_mag change at hit             */
    float    ax;                /* raw accel X at impact (g)           */
    float    ay;                /* raw accel Y at impact (g)           */
} dp_hit_event_t;

/* Pipeline state — treat as opaque, call dp_init() before use */
typedef struct {
    /* orientation */
    float    pitch;
    float    roll;

    /* low-pass filtered accel magnitude for hit detection */
    float    accel_mag_lpf;
    float    prev_accel_mag;

    /* hit state */
    int64_t  last_hit_us;
    int64_t  freeze_until_us;   /* suppress accel correction until this time */
    bool     initialized;       /* false until first sample sets baseline */

    /* ring buffer */
    dp_sample_t ring[DP_RING_SIZE];
    int          ring_head;     /* next write index */
    int          ring_count;    /* how many valid entries */
} dp_state_t;

/* ========================================================================
 *  API
 * ======================================================================== */

/* Initialise pipeline state. Call once at startup. */
void dp_init(dp_state_t *s);

/* Feed one raw IMU reading. Returns true if a hit was detected.
 * If hit is detected and `hit_out` is not NULL, features are written there.
 *
 *  raw_ax … raw_gz : int16_t straight from the IMU register read.
 *  now_us           : current timestamp in microseconds.
 */
bool dp_update(dp_state_t *s,
               int16_t raw_ax, int16_t raw_ay, int16_t raw_az,
               int16_t raw_gx, int16_t raw_gy, int16_t raw_gz,
               int64_t now_us,
               dp_hit_event_t *hit_out);

/* Read current orientation (valid after at least one dp_update). */
void dp_get_orientation(const dp_state_t *s, float *pitch, float *roll);

/* Format a debug line for serial output.
 * buf must be at least 128 bytes. Returns number of chars written. */
int dp_debug_line(const dp_state_t *s, const dp_sample_t *latest,
                  bool hit, char *buf, int buf_len);

#ifdef __cplusplus
}
#endif

#endif /* DRUM_PIPELINE_H */
