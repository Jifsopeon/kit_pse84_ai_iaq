#include "sen66_prediction.h"

#include "sen66_time_tree_model.h"

#include <stddef.h>

#define TIME_TREE_HISTORY_SIZE (31U)

typedef struct
{
    float pm25[TIME_TREE_HISTORY_SIZE];
    float pm10[TIME_TREE_HISTORY_SIZE];
    float voc[TIME_TREE_HISTORY_SIZE];
    float co2[TIME_TREE_HISTORY_SIZE];
    uint32_t total_count;
    uint8_t next_index;
    uint8_t latest_prediction;
} time_tree_history_t;

static time_tree_history_t time_tree_history;

static bool sen66_sample_is_valid(uint16_t pm1,
                                  uint16_t pm25,
                                  uint16_t pm4,
                                  uint16_t pm10,
                                  int16_t nox,
                                  uint16_t co2)
{
    return (pm1 < 65000U) &&
           (pm25 < 65000U) &&
           (pm4 < 65000U) &&
           (pm10 < 65000U) &&
           (nox < 30000) &&
           (co2 < 65000U);
}

static float history_get_age(const float *buffer, uint8_t age)
{
    uint8_t index = (uint8_t)((time_tree_history.next_index +
                               TIME_TREE_HISTORY_SIZE -
                               1U -
                               age) %
                              TIME_TREE_HISTORY_SIZE);

    return buffer[index];
}

static float history_mean_recent(const float *buffer, uint8_t window)
{
    float sum = 0.0f;
    uint8_t available = (time_tree_history.total_count < window)
                            ? (uint8_t)time_tree_history.total_count
                            : window;

    if (0U == available)
    {
        return 0.0f;
    }

    for (uint8_t i = 0U; i < available; i++)
    {
        sum += history_get_age(buffer, i);
    }

    return sum / (float)available;
}

static float history_max_recent(const float *buffer, uint8_t window)
{
    uint8_t available = (time_tree_history.total_count < window)
                            ? (uint8_t)time_tree_history.total_count
                            : window;

    if (0U == available)
    {
        return 0.0f;
    }

    float max_value = history_get_age(buffer, 0U);
    for (uint8_t i = 1U; i < available; i++)
    {
        float value = history_get_age(buffer, i);

        if (value > max_value)
        {
            max_value = value;
        }
    }

    return max_value;
}

static float history_delta(const float *buffer, float current_value, uint8_t window)
{
    if (time_tree_history.total_count <= window)
    {
        return 0.0f;
    }

    return current_value - history_get_age(buffer, window);
}

static void time_tree_history_add(float pm25,
                                  float pm10,
                                  float voc,
                                  float co2)
{
    time_tree_history.pm25[time_tree_history.next_index] = pm25;
    time_tree_history.pm10[time_tree_history.next_index] = pm10;
    time_tree_history.voc[time_tree_history.next_index] = voc;
    time_tree_history.co2[time_tree_history.next_index] = co2;

    time_tree_history.next_index =
        (uint8_t)((time_tree_history.next_index + 1U) %
                  TIME_TREE_HISTORY_SIZE);

    time_tree_history.total_count++;
}

void sen66_prediction_init(void)
{
    time_tree_history.total_count = 0U;
    time_tree_history.next_index = 0U;
    time_tree_history.latest_prediction = SEN66_PREDICTION_NOT_READY;
}

bool sen66_prediction_update(uint16_t pm1,
                             uint16_t pm25,
                             uint16_t pm4,
                             uint16_t pm10,
                             int16_t voc,
                             int16_t nox,
                             uint16_t co2,
                             uint8_t *prediction)
{
    if (!sen66_sample_is_valid(pm1, pm25, pm4, pm10, nox, co2))
    {
        if (NULL != prediction)
        {
            *prediction = time_tree_history.latest_prediction;
        }
        return false;
    }

    float pm1_0 = pm1 / 10.0f;
    float pm2_5 = pm25 / 10.0f;
    float pm4_0 = pm4 / 10.0f;
    float pm10_0 = pm10 / 10.0f;
    float voc_index = voc / 10.0f;
    float nox_index = nox / 10.0f;
    float co2_ppm = (float)co2;

    time_tree_history_add(pm2_5, pm10_0, voc_index, co2_ppm);

    bool smoking = sen66_time_tree_predict_smoking(
        pm1_0,
        pm2_5,
        pm4_0,
        pm10_0,
        voc_index,
        nox_index,
        co2_ppm,

        history_mean_recent(time_tree_history.pm25, 5U),
        history_max_recent(time_tree_history.pm25, 5U),
        history_delta(time_tree_history.pm25, pm2_5, 5U),
        history_mean_recent(time_tree_history.pm25, 10U),
        history_max_recent(time_tree_history.pm25, 10U),
        history_delta(time_tree_history.pm25, pm2_5, 10U),
        history_mean_recent(time_tree_history.pm25, 30U),
        history_max_recent(time_tree_history.pm25, 30U),
        history_delta(time_tree_history.pm25, pm2_5, 30U),

        history_mean_recent(time_tree_history.pm10, 5U),
        history_max_recent(time_tree_history.pm10, 5U),
        history_delta(time_tree_history.pm10, pm10_0, 5U),
        history_mean_recent(time_tree_history.pm10, 10U),
        history_max_recent(time_tree_history.pm10, 10U),
        history_delta(time_tree_history.pm10, pm10_0, 10U),
        history_mean_recent(time_tree_history.pm10, 30U),
        history_max_recent(time_tree_history.pm10, 30U),
        history_delta(time_tree_history.pm10, pm10_0, 30U),

        history_mean_recent(time_tree_history.voc, 5U),
        history_max_recent(time_tree_history.voc, 5U),
        history_delta(time_tree_history.voc, voc_index, 5U),
        history_mean_recent(time_tree_history.voc, 10U),
        history_max_recent(time_tree_history.voc, 10U),
        history_delta(time_tree_history.voc, voc_index, 10U),
        history_mean_recent(time_tree_history.voc, 30U),
        history_max_recent(time_tree_history.voc, 30U),
        history_delta(time_tree_history.voc, voc_index, 30U),

        history_mean_recent(time_tree_history.co2, 5U),
        history_max_recent(time_tree_history.co2, 5U),
        history_delta(time_tree_history.co2, co2_ppm, 5U),
        history_mean_recent(time_tree_history.co2, 10U),
        history_max_recent(time_tree_history.co2, 10U),
        history_delta(time_tree_history.co2, co2_ppm, 10U),
        history_mean_recent(time_tree_history.co2, 30U),
        history_max_recent(time_tree_history.co2, 30U),
        history_delta(time_tree_history.co2, co2_ppm, 30U));

    time_tree_history.latest_prediction = smoking ?
                                          SEN66_PREDICTION_SMOKING :
                                          SEN66_PREDICTION_NO_SMOKING;
    if (NULL != prediction)
    {
        *prediction = time_tree_history.latest_prediction;
    }

    return true;
}

uint8_t sen66_prediction_get_latest(void)
{
    return time_tree_history.latest_prediction;
}

const char *sen66_prediction_format(uint8_t prediction)
{
    switch (prediction)
    {
        case SEN66_PREDICTION_NO_SMOKING:
            return "no smoking";
        case SEN66_PREDICTION_SMOKING:
            return "smoking";
        default:
            return "Not Ready";
    }
}
