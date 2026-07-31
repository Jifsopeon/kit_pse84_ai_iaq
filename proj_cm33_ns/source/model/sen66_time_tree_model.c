#include "sen66_time_tree_model.h"

bool sen66_time_tree_predict_smoking(float pm1_0,
                                float pm2_5,
                                float pm4_0,
                                float pm10_0,
                                float voc_index,
                                float nox_index,
                                float co2_ppm,
                                float pm2_5_mean_5,
                                float pm2_5_max_5,
                                float pm2_5_delta_5,
                                float pm2_5_mean_10,
                                float pm2_5_max_10,
                                float pm2_5_delta_10,
                                float pm2_5_mean_30,
                                float pm2_5_max_30,
                                float pm2_5_delta_30,
                                float pm10_0_mean_5,
                                float pm10_0_max_5,
                                float pm10_0_delta_5,
                                float pm10_0_mean_10,
                                float pm10_0_max_10,
                                float pm10_0_delta_10,
                                float pm10_0_mean_30,
                                float pm10_0_max_30,
                                float pm10_0_delta_30,
                                float voc_index_mean_5,
                                float voc_index_max_5,
                                float voc_index_delta_5,
                                float voc_index_mean_10,
                                float voc_index_max_10,
                                float voc_index_delta_10,
                                float voc_index_mean_30,
                                float voc_index_max_30,
                                float voc_index_delta_30,
                                float co2_ppm_mean_5,
                                float co2_ppm_max_5,
                                float co2_ppm_delta_5,
                                float co2_ppm_mean_10,
                                float co2_ppm_max_10,
                                float co2_ppm_delta_10,
                                float co2_ppm_mean_30,
                                float co2_ppm_max_30,
                                float co2_ppm_delta_30)
{
    if (pm2_5_mean_5 <= 9.095145f)
    {
        if (co2_ppm_max_30 <= 431.000000f)
        {
            if (co2_ppm_max_30 <= 430.500000f)
            {
                if (voc_index_delta_5 <= -0.500000f)
                {
                    return false;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return true;
            }
        }
        else
        {
            if (pm1_0 <= 0.050000f)
            {
                return false;
            }
            else
            {
                return false;
            }
        }
    }
    else
    {
        if (co2_ppm_max_30 <= 589.000000f)
        {
            if (co2_ppm_delta_30 <= 4.500000f)
            {
                if (pm1_0 <= 4.800000f)
                {
                    return false;
                }
                else
                {
                    if (co2_ppm_delta_5 <= 2.500000f)
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                if (pm10_0_mean_30 <= 14.776407f)
                {
                    if (pm2_5_mean_5 <= 9.770000f)
                    {
                        return false;
                    }
                    else
                    {
                        return true;
                    }
                }
                else
                {
                    return false;
                }
            }
        }
        else
        {
            if (pm10_0_mean_30 <= 19.958611f)
            {
                if (pm1_0 <= 5.500000f)
                {
                    if (co2_ppm <= 612.000000f)
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }
            else
            {
                if (pm2_5 <= 18.500000f)
                {
                    if (co2_ppm_delta_30 <= -13.000000f)
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    if (pm10_0_mean_30 <= 40.297601f)
                    {
                        return false;
                    }
                    else
                    {
                        return true;
                    }
                }
            }
        }
    }
}
