/*
 * subjective_score.h
 * 客观物理量 -> 主观舒适度评分 的映射公式
 *
 * v3 更新：新增"启停冲击持续时间"评分维度（对应客户需求文档里提到的
 * "启停冲击持续时间"特征参数）。冲击持续时间越长，说明电梯启停过程
 * 越拖沓/不干脆，乘客体感越差，所以也是越长分越低。
 *
 * 设计说明：
 *   这里的公式（高斯函数中心点、指数衰减系数、分段线性拐点）是基于常识和查阅到的
 *   一般舒适度评价资料估算出的初始参数，不是从某一份具体国标逐字抄来的精确公式。
 *   写论文时建议表述为"参考相关舒适度评价研究，构建的映射模型"，
 *   如果你的指导老师要求对应具体国标条款，需要你再对照 GB/T 24474（电梯乘运质量）
 *   等标准核实调整这里的参数，本文件中所有可调参数都提取成了宏定义，方便你后续单独调整。
 *
 *   本模块是纯计算函数（不依赖FreeRTOS/ESP-IDF），可以直接拿去PC上用gcc编译测试，
 *   验证公式效果后再集成进单片机代码。
 */

#ifndef SUBJECTIVE_SCORE_H
#define SUBJECTIVE_SCORE_H

// ---------- 单项评分的可调参数 ----------

// 振动舒适度：振动幅度(m/s^2)越大分越低，指数衰减
float score_vibration(float max_amplitude);

// 平稳感（冲击/顿挫感）：jerk即相邻两次采样加速度变化量(m/s^2)，越大分越低
float score_smoothness(float max_jerk);

// 新增：冲击持续时间(ms)评分。启停时加速度持续偏离基准超过阈值的最长连续时长，
// 时间越长说明启停过程越拖沓，乘客体感越不干脆利落。
#define SHOCK_DURATION_GOOD_MS   150.0f   // 150ms以内认为是干脆利落的正常启停，满分
#define SHOCK_DURATION_BAD_MS    800.0f   // 超过800ms认为拖沓明显，趋近0分
float score_shock_duration(float shock_duration_ms);

// 噪声舒适度
float score_noise(float avg_db);

// 温度舒适度：高斯函数
float score_thermal(float avg_temp);

// 湿度舒适度：高斯函数
float score_humidity(float avg_humidity);

// ---------- 6维主观评分结构体（新增冲击持续时间一项） ----------
typedef struct {
    float vibration_score;      // 振动舒适度 (0-10)
    float smoothness_score;     // 平稳感/冲击感 (0-10)
    float shock_duration_score; // 冲击持续时间评分 (0-10) —— 新增
    float noise_score;          // 噪声舒适度 (0-10)
    float thermal_score;        // 温度舒适度 (0-10)
    float humidity_score;       // 湿度舒适度 (0-10)
    float overall_score;        // 加权综合分 (0-10)
} subjective_scores_t;

// ---------- 综合评分 ----------
// 新增了 shock_duration_ms 参数（本次运行统计到的最长冲击持续时间，单位ms）
subjective_scores_t compute_subjective_scores(float max_amplitude, float max_jerk,
                                               float shock_duration_ms,
                                               float avg_db, float avg_temp, float avg_humidity);

// 综合分转等级文字。v3新增"危"这一级，对应客户要求的"优/良/中/差/危"五级分级
const char *score_to_grade(float overall_score);

#endif // SUBJECTIVE_SCORE_H