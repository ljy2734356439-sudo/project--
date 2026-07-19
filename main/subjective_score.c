/*
 * subjective_score.c
 * v3：在原有优化版公式基础上，新增冲击持续时间评分维度，权重重新分配；
 *     分级从四级(优良中差)扩展为五级(优良中差危)，对应客户需求文档要求。
 */

#include "subjective_score.h"
#include <math.h>

static float clamp_score(float s)
{
    if (s < 0.0f) return 0.0f;
    if (s > 10.0f) return 10.0f;
    return s;
}

float score_vibration(float max_amplitude)
{
    if (max_amplitude < 0) max_amplitude = -max_amplitude;
    // 当 max_amplitude = 4.7m/s^2 时，大约可以拿到 7.5 分
    float s = 10.0f * expf(-0.06f * max_amplitude);
    return clamp_score(s);
}

float score_smoothness(float max_jerk)
{
    if (max_jerk < 0) max_jerk = -max_jerk;
    // 当 max_jerk = 4.2m/s^2 时，大约可以拿到 7.2 分
    float s = 10.0f * expf(-0.08f * max_jerk);
    return clamp_score(s);
}

// 冲击持续时间评分：分段线性，150ms以内满分，超过800ms趋近0分
float score_shock_duration(float shock_duration_ms)
{
    if (shock_duration_ms < 0) shock_duration_ms = 0;
    if (shock_duration_ms <= SHOCK_DURATION_GOOD_MS) return 10.0f;
    if (shock_duration_ms >= SHOCK_DURATION_BAD_MS) return 0.0f;

    float ratio = (shock_duration_ms - SHOCK_DURATION_GOOD_MS)
                  / (SHOCK_DURATION_BAD_MS - SHOCK_DURATION_GOOD_MS);
    return clamp_score(10.0f * (1.0f - ratio));
}

float score_noise(float avg_db)
{
    // 电梯运行中，低于 52dB 都是非常安静的高级环境，直接给 10 分
    if (avg_db <= 52.0f) return 10.0f;
    // 对数柔和衰减，哪怕平均噪声达到 65dB，依然有 5.2 分左右，不会断崖跌零
    float diff = avg_db - 52.0f;
    float s = 10.0f * expf(-0.05f * diff);
    return clamp_score(s);
}

float score_thermal(float avg_temp)
{
    float diff = avg_temp - 25.0f;
    float s = 10.0f * expf(-(diff * diff) / (2.0f * 4.0f * 4.0f));
    return clamp_score(s);
}

float score_humidity(float avg_humidity)
{
    float diff = avg_humidity - 55.0f;
    float s = 10.0f * expf(-(diff * diff) / (2.0f * 15.0f * 15.0f));
    return clamp_score(s);
}

subjective_scores_t compute_subjective_scores(float max_amplitude, float max_jerk,
                                               float shock_duration_ms,
                                               float avg_db, float avg_temp, float avg_humidity)
{
    subjective_scores_t result;

    result.vibration_score      = score_vibration(max_amplitude);
    result.smoothness_score     = score_smoothness(max_jerk);
    result.shock_duration_score = score_shock_duration(shock_duration_ms);
    result.noise_score          = score_noise(avg_db);
    result.thermal_score        = score_thermal(avg_temp);
    result.humidity_score       = score_humidity(avg_humidity);

    // 权重配比：新增冲击持续时间维度后，从原来5项重新分配到6项，
    // 振动+平稳+冲击时间这三项都属于"运行动力学"范畴，合计权重0.60，
    // 跟客户文档里"启动加速度/振动峰峰值用于表征运行平稳感和振动舒适度"的思路一致。
    const float W_VIB   = 0.25f;
    const float W_SMOOTH = 0.20f;
    const float W_SHOCK  = 0.15f;
    const float W_NOISE  = 0.20f;
    const float W_TEMP   = 0.10f;
    const float W_HUMI   = 0.10f;
    // 六项权重之和 = 1.00

    result.overall_score = W_VIB * result.vibration_score
                          + W_SMOOTH * result.smoothness_score
                          + W_SHOCK * result.shock_duration_score
                          + W_NOISE * result.noise_score
                          + W_TEMP * result.thermal_score
                          + W_HUMI * result.humidity_score;

    result.overall_score = clamp_score(result.overall_score);

    return result;
}

// v3新增"危"这一级，对应客户要求的"优/良/中/差/危"五级分级
const char *score_to_grade(float overall_score)
{
    if (overall_score >= 8.5f) return "优";
    if (overall_score >= 6.8f) return "良";
    if (overall_score >= 4.8f) return "中";
    if (overall_score >= 2.5f) return "差";
    return "危";   // 低于2.5分，说明存在较严重的乘运质量问题，需要立即关注
}
