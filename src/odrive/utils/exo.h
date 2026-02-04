#ifndef EXO_NEW
#define EXO_NEW

#include <Arduino.h>

// #define CIRCULAR_INDEX(INDEX, WRITE_INDEX, COUNT) ((WRITE_INDEX - COUNT + INDEX + MAX_STAMPS) % MAX_STAMPS)

typedef struct
{
    float proj_angle_diff;
    float smooth_proj_angle_diff;
    unsigned long timestamp;
} stamp;

typedef struct
{
    stamp* stamps;
    int write_index;
    int stamp_count;
    
    int update_frequency_hz;
    int update_period_ms;
    int max_delay_ms;
    int max_stamp_count;
} stamp_tracker;

extern stamp_tracker tracker;

void exo_record_stamp(float angle)
{
    tracker.stamps[tracker.write_index].proj_angle_diff = angle;
    tracker.stamps[tracker.write_index].smooth_proj_angle_diff = (1 - 0.05) * tracker.stamps[(tracker.write_index - 1 + tracker.max_stamp_count) % tracker.max_stamp_count].smooth_proj_angle_diff + 0.05 * angle;
    tracker.stamps[tracker.write_index].timestamp = millis();

    // printf(">angle: %f\n", tracker.stamps[tracker.write_index].proj_angle_diff);
    // printf(">prev_smootH_angle: %f\n", tracker.stamps[(tracker.write_index - 1 + tracker.max_stamp_count) % tracker.max_stamp_count].smooth_proj_angle_diff);
    // printf(">smooth_angle: %f\n", tracker.stamps[tracker.write_index].smooth_proj_angle_diff);

    tracker.write_index = (tracker.write_index + 1) % tracker.max_stamp_count;
    if (tracker.stamp_count < tracker.max_stamp_count)
        tracker.stamp_count++;
}

float exo_update(float angle_R, float angle_L, int delay_ms, float gain)
{
    float proj_angle = sin(angle_R) - sin(angle_L);
    exo_record_stamp(proj_angle);

    if (tracker.stamp_count < tracker.max_stamp_count)
        return 0;

    int index_offset = delay_ms / tracker.update_period_ms;
    float output_iq = gain * tracker.stamps[(tracker.write_index - 1 - index_offset + tracker.max_stamp_count) % tracker.max_stamp_count].smooth_proj_angle_diff/2;
    // printf(">IQ: %f\n", output_iq);
    return output_iq;
}

void exo_init(int update_frequency_hz)
{
    tracker.update_frequency_hz = update_frequency_hz;
    tracker.update_period_ms = 1000 / update_frequency_hz;
    tracker.max_delay_ms = MAX_DELAY_MS;

    tracker.max_stamp_count = MAX_DELAY_MS / tracker.update_period_ms;
    tracker.stamps = (stamp *)calloc(tracker.max_stamp_count, sizeof(stamp));
    for (int i = 0; i < tracker.max_stamp_count; i++)
    {
        tracker.stamps[i].proj_angle_diff = 0;
        tracker.stamps[i].smooth_proj_angle_diff = 0;
        tracker.stamps[i].timestamp = 0;
    }
    
    tracker.write_index = 0;
    tracker.stamp_count = 0;
}

#endif