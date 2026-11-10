#include "track.h"
#include "line_sensor.h"
#include "motor.h"
#include "imu.h"
#include "link.h"
#include <math.h>

/* ========================================================================== */
/* 参考 25-aiming-cart 的循迹参数                                             */
/* ========================================================================== */

#define TRACK_PI                    3.14159265358979323846f
#define TRACK_DEG_TO_RAD            0.01745329251994329577f
#define TRACK_RAD_TO_DEG            57.295779513082320876f

#define TRACK_STRAIGHT_MPS          0.290f
#define TRACK_TURN_MPS              0.140f
#define TRACK_ACCEL_MPS2            0.800f
#define TRACK_DECEL_MPS2            0.600f
#define TRACK_ANGULAR_ACCEL_RAD2    8.0f
#define TRACK_LINE_W_ACCEL_RAD2     3.0f

#define TRACK_LINE_KP               0.018f
#define TRACK_LINE_KD               0.004f
#define TRACK_LINE_KH               0.40f
#define TRACK_LINE_KI               0.010f
#define TRACK_LINE_I_LIMIT          40.0f
#define TRACK_YAW_RATE_KG           0.80f
#define TRACK_YAW_CORRECTION_MAX    1.0f
#define TRACK_LINE_ERROR_TAU_SEC    0.035f

#define TRACK_PROBE_SPACING_MM      11.14f
#define TRACK_SENSOR_AHEAD_M        0.185f
#define TRACK_RADIUS_M              0.062f
#define TRACK_WHEEL_BASE_M          0.115f

#define TRACK_TURN_LAG_SEC          0.20f
#define TRACK_TURN_STOP_LEAD_SEC    0.08f
#define TRACK_TURN_FAIL_RAD         (130.0f * TRACK_DEG_TO_RAD)
#define TRACK_TURN_WRONG_RAD        (-15.0f * TRACK_DEG_TO_RAD)
#define TRACK_TURN_TIMEOUT_SEC      5.0f
#define TRACK_PRE_TIMEOUT_SEC       2.0f

#define TRACK_HEADING_TRIM_TAU_SEC  0.8f
#define TRACK_HEADING_TRIM_ERROR_MM (1.0f * TRACK_PROBE_SPACING_MM)

#define TRACK_POST_TURN_RAMP_SEC     0.80f
#define TRACK_POST_TURN_KP_SCALE     0.60f
#define TRACK_POST_TURN_KD_SCALE     1.25f
#define TRACK_POST_TURN_KI_SCALE     0.25f
#define TRACK_POST_TURN_KH_SCALE     0.50f
#define TRACK_POST_TURN_W_LIMIT      0.80f

#define TRACK_LINE_LOST_SPEED_MPS   0.080f
#define TRACK_LINE_LOST_TIMEOUT_SEC 0.20f
#define TRACK_LOST_TIMEOUT_MS       700u
#define TRACK_SEARCH_SIDE_FRACTION  0.5f
#define TRACK_SEARCH_W_RAD_S        1.0f

#define TRACK_CORNER_CONFIRM_FRAMES 3u
#define TRACK_REDETECT_FRAMES       2u

/* 电机 100% 对应的轮子线速度(m/s)。 */
#define TRACK_MPS_AT_FULL           (MOTOR_RPM_AT_FULL / 60.0f * 6.2831853f * \
                                     (MOTOR_WHEEL_RADIUS_CM / 100.0f))
#define TRACK_MPS_PER_PCT           (TRACK_MPS_AT_FULL / 100.0f)

/* 转弯内轮至少保持约 1 cm/s, 与参考底盘的量级一致。 */
#define TRACK_TURN_INNER_MIN_MPS    0.010f
#define TRACK_LINE_INNER_MIN_MPS    0.010f

/* ========================================================================== */
/* 内部状态                                                                   */
/* ========================================================================== */

static TrackState state;

static uint16_t sensor_frames_prev;
static uint8_t  sensor_frame_new;
static uint8_t  corner_confirm;
static uint8_t  line_redetect;
static uint8_t  line_error_initialized;

static float line_error_mm;
static float line_error_prev_mm;
static float line_derivative_mm_s;
static float line_integral_mm_s;
static float last_line_error_mm;
static uint8_t post_turn_ramp_active;

static float body_heading_rad;
static float line_direction_rad;
static float heading_trim_rad;

static float body_v_mps;
static float body_w_rad_s;
static float measured_v_mps;
static float wheel_target_l_mps;
static float wheel_target_r_mps;

static float state_time_s;
static float state_distance_m;
static float lost_time_s;
static uint32_t lost_tick;

static uint32_t last_odo_l_cm;
static uint32_t last_odo_r_cm;
static float left_delta_m;
static float right_delta_m;

static int8_t turn_dir;
static float turn_target_deg;
static float turn_angle_rad;
static float turn_elapsed_s;
static float turn_arc_m;

static uint8_t last_turn_valid;
static float last_turn_arc_cm;
static float last_turn_yaw_deg;
static float last_turn_time_s;

static float track_approach(float value, float target, float max_step)
{
    if (target > (value + max_step)) {
        return value + max_step;
    }
    if (target < (value - max_step)) {
        return value - max_step;
    }
    return target;
}

static float track_clampf(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

/* ========================================================================== */
/* 灰度位置和直角                                                             */
/* ========================================================================== */

static uint8_t track_line_error_mm(uint8_t mask, float *error_mm)
{
    uint8_t i;
    uint8_t first = LINE_CH_NUM;
    uint8_t last = 0u;
    uint8_t count = 0u;
    float sum = 0.0f;

    for (i = 0u; i < LINE_CH_NUM; i++) {
        if ((mask & (uint8_t)(1u << i)) != 0u) {
            if (first == LINE_CH_NUM) {
                first = i;
            }
            last = i;
            count++;
            sum += ((float)i - 3.5f) * TRACK_PROBE_SPACING_MM;
        }
    }
    if ((count == 0u) || (count > 3u) || ((uint8_t)(last - first + 1u) != count)) {
        return 0u;
    }
    *error_mm = sum / (float)count;
    return 1u;
}

static uint8_t track_corner(uint8_t mask, int8_t *dir)
{
    uint8_t left = LineSensor_PopCount((uint8_t)(mask & 0x0Fu));
    uint8_t right = LineSensor_PopCount((uint8_t)(mask & 0xF0u));

    if (LineSensor_PopCount(mask) == LINE_CH_NUM) {
        return 0u;
    }
    if ((left >= 3u) && (right <= 2u)) {
        *dir = -1;
        return 1u;
    }
    if ((right >= 3u) && (left <= 2u)) {
        *dir = +1;
        return 1u;
    }
    return 0u;
}

static float track_search_w(float error_mm)
{
    float side = TRACK_SEARCH_SIDE_FRACTION * TRACK_PROBE_SPACING_MM;

    if (error_mm <= -side) {
        return TRACK_SEARCH_W_RAD_S;
    }
    if (error_mm >= side) {
        return -TRACK_SEARCH_W_RAD_S;
    }
    return 0.0f;
}

static float track_heading_error(float edge_direction_rad)
{
    return body_heading_rad - edge_direction_rad - heading_trim_rad;
}

/* ========================================================================== */
/* v/w -> 左右轮目标                                                          */
/* ========================================================================== */

static void track_apply_velocity(float v_mps, float w_rad_s, float min_wheel_mps)
{
    float half_diff = 0.5f * TRACK_WHEEL_BASE_M * w_rad_s;
    float left = v_mps - half_diff;
    float right = v_mps + half_diff;
    float peak;
    float scale;

    /*
     * 峰值超过轮速上限时按比例缩放, 保持 v/w 的曲率关系。
     */
    peak = fmaxf(fabsf(left), fabsf(right));
    if (peak > TRACK_MPS_AT_FULL) {
        scale = TRACK_MPS_AT_FULL / peak;
        left *= scale;
        right *= scale;
    }

    left = track_clampf(left, min_wheel_mps, TRACK_MPS_AT_FULL);
    right = track_clampf(right, min_wheel_mps, TRACK_MPS_AT_FULL);
    wheel_target_l_mps = left;
    wheel_target_r_mps = right;
    Motor_SetTargetPercent(MOTOR_LEFT,  left / TRACK_MPS_PER_PCT);
    Motor_SetTargetPercent(MOTOR_RIGHT, right / TRACK_MPS_PER_PCT);
}

/* ========================================================================== */
/* 状态切换                                                                   */
/* ========================================================================== */

static void track_reset_line_filter(void)
{
    line_error_initialized = 0u;
    line_derivative_mm_s = 0.0f;
    line_integral_mm_s = 0.0f;
}

static void track_enter_line(void)
{
    state = TRACK_LINE;
    state_time_s = 0.0f;
    state_distance_m = 0.0f;
    lost_time_s = 0.0f;
    corner_confirm = 0u;
    line_redetect = 0u;
    track_reset_line_filter();
}

static void track_enter_pre_turn(int8_t dir)
{
    state = TRACK_PRE_TURN;
    state_time_s = 0.0f;
    state_distance_m = 0.0f;
    corner_confirm = 0u;
    /*
     * track_corner(): 左=-1, 右=+1。
     * v/w 差速约定: 左转 w>0, 右转 w<0。这里转换成角速度符号。
     */
    turn_dir = (int8_t)(-dir);
    turn_target_deg = (turn_dir > 0) ? 90.0f : -90.0f;
}

static void track_enter_turn(void)
{
    state = TRACK_TURN;
    state_time_s = 0.0f;
    state_distance_m = 0.0f;
    turn_angle_rad = 0.0f;
    turn_elapsed_s = 0.0f;
    turn_arc_m = 0.0f;
    track_reset_line_filter();
}

static void track_enter_lost(void)
{
    state = TRACK_LOST;
    lost_time_s = 0.0f;
    lost_tick = HAL_GetTick();
    line_redetect = 0u;
    post_turn_ramp_active = 0u;
    track_reset_line_filter();
}

static void track_stop(void)
{
    state = TRACK_STOP;
    body_v_mps = 0.0f;
    body_w_rad_s = 0.0f;
    post_turn_ramp_active = 0u;
    Motor_Stop();
}

static void track_record_turn(void)
{
    last_turn_valid = 1u;
    last_turn_arc_cm = turn_arc_m * 100.0f;
    last_turn_yaw_deg = turn_angle_rad * TRACK_RAD_TO_DEG;
    last_turn_time_s = turn_elapsed_s;
}

/* ========================================================================== */
/* 里程、IMU 和每周期入口                                                     */
/* ========================================================================== */

static float track_update_odometry(float dt)
{
    uint32_t odo_l = (uint32_t)Motor_GetDistanceCm(MOTOR_LEFT);
    uint32_t odo_r = (uint32_t)Motor_GetDistanceCm(MOTOR_RIGHT);
    int32_t delta_l_cm = (int32_t)(odo_l - last_odo_l_cm);
    int32_t delta_r_cm = (int32_t)(odo_r - last_odo_r_cm);
    float raw_v;

    last_odo_l_cm = odo_l;
    last_odo_r_cm = odo_r;
    left_delta_m  = (float)delta_l_cm * 0.01f;
    right_delta_m = (float)delta_r_cm * 0.01f;

    raw_v = 0.5f * (left_delta_m + right_delta_m) / dt;
    measured_v_mps += dt / (0.03f + dt) * (raw_v - measured_v_mps);
    state_distance_m += 0.5f * (left_delta_m + right_delta_m);
    if (state == TRACK_TURN) {
        turn_arc_m += 0.5f * (left_delta_m + right_delta_m);
    }
    return raw_v;
}

static float track_get_yaw_rate(uint8_t *valid)
{
    if (IMU_IsOnline() != 0u) {
        *valid = 1u;
        return IMU_GetGyroZDps() * TRACK_DEG_TO_RAD;
    }

    /* 参考实现的降级策略: 用左右轮速度差反推车体角速度。 */
    *valid = 1u;
    return (right_delta_m - left_delta_m) /
           (TRACK_WHEEL_BASE_M * MOTOR_CTRL_DT);
}

/* ========================================================================== */
/* 主状态机                                                                   */
/* ========================================================================== */

void Track_Update(void)
{
    uint8_t mask = LineSensor_GetMask();
    uint8_t cnt = LineSensor_PopCount(mask);
    uint16_t sensor_frames = LineSensor_GetFrameCount();
    uint8_t normal;
    uint8_t gyro_valid;
    int8_t corner_dir = 0;
    float gyro_z_rad_s;
    float target_v;
    float target_w;
    float w_accel;
    float error_mm = 0.0f;
    float heading_error;
    float wheel_w;
    float remaining;
    float turn_rate;
    const float dt = MOTOR_CTRL_DT;

    sensor_frame_new = (sensor_frames != sensor_frames_prev) ? 1u : 0u;
    sensor_frames_prev = sensor_frames;

    if (Motor_IsEnabled() == 0u) {
        track_stop();
        return;
    }
    if (LineSensor_IsOnline() == 0u) {
        track_stop();
        return;
    }

    (void)track_update_odometry(dt);
    gyro_z_rad_s = track_get_yaw_rate(&gyro_valid);
    if (gyro_valid == 0u) {
        track_stop();
        return;
    }
    body_heading_rad += gyro_z_rad_s * dt;

    if (state == TRACK_STOP) {
        if (cnt != 0u) {
            if (++line_redetect >= TRACK_REDETECT_FRAMES) {
                body_v_mps = 0.0f;
                body_w_rad_s = 0.0f;
                track_enter_line();
            }
        } else {
            line_redetect = 0u;
        }
        Motor_Stop();
        return;
    }

    normal = track_line_error_mm(mask, &error_mm);
    if (cnt != 0u) {
        last_line_error_mm = error_mm;
    }

    target_v = TRACK_STRAIGHT_MPS;
    target_w = 0.0f;
    w_accel = TRACK_LINE_W_ACCEL_RAD2;
    state_time_s += dt;

    if ((state == TRACK_LINE) && (post_turn_ramp_active != 0u)) {
        float ramp = state_time_s / TRACK_POST_TURN_RAMP_SEC;
        if (ramp >= 1.0f) {
            ramp = 1.0f;
            post_turn_ramp_active = 0u;
        }
        target_v = TRACK_TURN_MPS +
                   (TRACK_STRAIGHT_MPS - TRACK_TURN_MPS) * ramp;
    }

    if (state == TRACK_LINE) {
        if ((sensor_frame_new != 0u) && (track_corner(mask, &corner_dir) != 0u)) {
            if (++corner_confirm >= TRACK_CORNER_CONFIRM_FRAMES) {
                track_enter_pre_turn(corner_dir);
            }
        } else if (sensor_frame_new != 0u) {
            corner_confirm = 0u;
        }

        if (state == TRACK_LINE) {
            if (normal != 0u) {
                float previous;
                float raw_derivative;

                if (line_error_initialized == 0u) {
                    line_error_mm = error_mm;
                    line_error_prev_mm = error_mm;
                    line_error_initialized = 1u;
                } else {
                    previous = line_error_mm;
                    line_error_mm +=
                        dt / (TRACK_LINE_ERROR_TAU_SEC + dt) *
                        (error_mm - line_error_mm);
                    raw_derivative = (line_error_mm - previous) / dt;
                    line_derivative_mm_s +=
                        dt / (0.03f + dt) * (raw_derivative - line_derivative_mm_s);
                }

                heading_error = track_heading_error(line_direction_rad);
                line_integral_mm_s += line_error_mm * dt;
                line_integral_mm_s = track_clampf(
                    line_integral_mm_s,
                    -TRACK_LINE_I_LIMIT,
                    TRACK_LINE_I_LIMIT);
                {
                    float kp = TRACK_LINE_KP;
                    float kd = TRACK_LINE_KD;
                    float ki = TRACK_LINE_KI;
                    float kh = TRACK_LINE_KH;
                    float w_limit = 1.5f;

                    if (post_turn_ramp_active != 0u) {
                        kp *= TRACK_POST_TURN_KP_SCALE;
                        kd *= TRACK_POST_TURN_KD_SCALE;
                        ki *= TRACK_POST_TURN_KI_SCALE;
                        kh *= TRACK_POST_TURN_KH_SCALE;
                        w_limit = TRACK_POST_TURN_W_LIMIT;
                    }
                    target_w = track_clampf(
                        -kp * line_error_mm
                        - kd * line_derivative_mm_s
                        - ki * line_integral_mm_s
                        - kh * heading_error,
                        -w_limit, w_limit);
                }

                if (fabsf(line_error_mm) < TRACK_HEADING_TRIM_ERROR_MM) {
                    heading_trim_rad +=
                        dt / (TRACK_HEADING_TRIM_TAU_SEC + dt) * heading_error;
                }

                lost_time_s = 0.0f;
            } else {
                lost_time_s += dt;
                target_v = TRACK_LINE_LOST_SPEED_MPS;
                /* 转弯刚结束短暂看不到线时, 用新直边航向继续找线。 */
                target_w = track_clampf(
                    -TRACK_LINE_KH * track_heading_error(line_direction_rad),
                    -1.5f, 1.5f);
                if ((mask == 0u) && (line_error_initialized != 0u)) {
                    target_w = track_search_w(line_error_mm);
                }
                if (lost_time_s > TRACK_LINE_LOST_TIMEOUT_SEC) {
                    track_enter_lost();
                }
            }

        }
    }

    if (state == TRACK_PRE_TURN) {
        float w_turn = TRACK_TURN_MPS / TRACK_RADIUS_M;
        float turn_lead = measured_v_mps *
            (TRACK_TURN_LAG_SEC + w_turn / (2.0f * TRACK_ANGULAR_ACCEL_RAD2));

        target_v = TRACK_TURN_MPS;
        target_w = 0.0f;
        remaining = TRACK_SENSOR_AHEAD_M - TRACK_RADIUS_M -
                    turn_lead - state_distance_m;

        if ((remaining <= 0.0f) &&
            (state_time_s >= 0.06f) &&
            (fabsf(body_w_rad_s) < 0.05f)) {
            track_enter_turn();
        }
        if (state_time_s > TRACK_PRE_TIMEOUT_SEC) {
            track_enter_lost();
        }
    }

    if (state == TRACK_TURN) {
        float signed_gyro = (float)turn_dir * gyro_z_rad_s;

        turn_elapsed_s += dt;
        turn_angle_rad += signed_gyro * dt;
        target_v = TRACK_TURN_MPS;
        w_accel = TRACK_ANGULAR_ACCEL_RAD2;

        remaining = TRACK_PI * 0.5f - turn_angle_rad -
                    signed_gyro * TRACK_TURN_LAG_SEC;
        if (remaining < 0.0f) {
            remaining = 0.0f;
        }
        turn_rate = fminf(
            fmaxf(body_v_mps, TRACK_TURN_MPS) / TRACK_RADIUS_M,
            sqrtf(2.0f * TRACK_ANGULAR_ACCEL_RAD2 * remaining));
        target_w = (float)turn_dir * turn_rate;

        /*
         * 转弯曲率由位置层立即建立, 不再等待8 rad/s²的w斜坡。
         * 左右轮目标速度仍会被电机目标斜率和速度PID平滑。
         */
        body_w_rad_s = target_w;

        if ((turn_angle_rad > TRACK_TURN_FAIL_RAD) ||
            (turn_angle_rad < TRACK_TURN_WRONG_RAD) ||
            (turn_elapsed_s > TRACK_TURN_TIMEOUT_SEC)) {
            track_record_turn();
            track_stop();
            return;
        }

        if ((turn_angle_rad +
             fmaxf(signed_gyro, 0.0f) * TRACK_TURN_STOP_LEAD_SEC) >=
            (TRACK_PI * 0.5f)) {
            line_direction_rad += (float)turn_dir * TRACK_PI * 0.5f;
            track_record_turn();
            track_enter_line();
            post_turn_ramp_active = 1u;
            target_v = TRACK_TURN_MPS;
            target_w = 0.0f;
        }
    }

    if (state == TRACK_LOST) {
        target_v = TRACK_LINE_LOST_SPEED_MPS;
        target_w = track_search_w(last_line_error_mm);
        w_accel = TRACK_LINE_W_ACCEL_RAD2;

        if ((sensor_frame_new != 0u) && (normal != 0u)) {
            if (++line_redetect >= TRACK_REDETECT_FRAMES) {
                track_enter_line();
                target_w = 0.0f;
            }
        } else if (sensor_frame_new != 0u) {
            line_redetect = 0u;
        }

        if ((HAL_GetTick() - lost_tick) > TRACK_LOST_TIMEOUT_MS) {
            track_stop();
            return;
        }
    }

    body_v_mps = track_approach(
        body_v_mps, target_v,
        ((target_v > body_v_mps) ? TRACK_ACCEL_MPS2 : TRACK_DECEL_MPS2) * dt);
    body_w_rad_s = track_approach(body_w_rad_s, target_w, w_accel * dt);

    /* 参考实现的直线角速度反馈: 仅直线和预转弯使用。 */
    wheel_w = body_w_rad_s;
    if ((state == TRACK_LINE) || (state == TRACK_PRE_TURN)) {
        wheel_w += track_clampf(
            TRACK_YAW_RATE_KG * (body_w_rad_s - gyro_z_rad_s),
            -TRACK_YAW_CORRECTION_MAX, TRACK_YAW_CORRECTION_MAX);
    }

    track_apply_velocity(body_v_mps, wheel_w,
                         (state == TRACK_TURN) ?
                         TRACK_TURN_INNER_MIN_MPS : TRACK_LINE_INNER_MIN_MPS);
}

void Track_Init(void)
{
    state = TRACK_LINE;
    sensor_frames_prev = LineSensor_GetFrameCount();
    sensor_frame_new = 0u;
    corner_confirm = 0u;
    line_redetect = 0u;
    line_error_initialized = 0u;
    line_error_mm = 0.0f;
    line_error_prev_mm = 0.0f;
    line_derivative_mm_s = 0.0f;
    line_integral_mm_s = 0.0f;
    last_line_error_mm = 0.0f;
    post_turn_ramp_active = 0u;
    body_heading_rad = 0.0f;
    line_direction_rad = 0.0f;
    heading_trim_rad = 0.0f;
    body_v_mps = 0.0f;
    body_w_rad_s = 0.0f;
    measured_v_mps = 0.0f;
    wheel_target_l_mps = 0.0f;
    wheel_target_r_mps = 0.0f;
    state_time_s = 0.0f;
    state_distance_m = 0.0f;
    lost_time_s = 0.0f;
    lost_tick = 0u;
    last_odo_l_cm = (uint32_t)Motor_GetDistanceCm(MOTOR_LEFT);
    last_odo_r_cm = (uint32_t)Motor_GetDistanceCm(MOTOR_RIGHT);
    left_delta_m = 0.0f;
    right_delta_m = 0.0f;
    turn_dir = 0;
    turn_target_deg = 0.0f;
    turn_angle_rad = 0.0f;
    turn_elapsed_s = 0.0f;
    turn_arc_m = 0.0f;
    last_turn_valid = 0u;
    last_turn_arc_cm = 0.0f;
    last_turn_yaw_deg = 0.0f;
    last_turn_time_s = 0.0f;
}

TrackState Track_GetState(void)
{
    return state;
}

float Track_GetError(void)
{
    return line_error_mm;
}

float Track_GetTurnTargetDeg(void)
{
    return turn_target_deg;
}

int8_t Track_GetTurnDir(void)
{
    return turn_dir;
}

float Track_GetBodyTargetMps(void)
{
    return body_v_mps;
}

float Track_GetBodyTargetRadps(void)
{
    return body_w_rad_s;
}

float Track_GetBodyMeasuredMps(void)
{
    return measured_v_mps;
}

float Track_GetTurnAngleDeg(void)
{
    return turn_angle_rad * TRACK_RAD_TO_DEG;
}

float Track_GetHeadingDeg(void)
{
    return body_heading_rad * TRACK_RAD_TO_DEG;
}

float Track_GetLeftTargetMps(void)
{
    return wheel_target_l_mps;
}

float Track_GetRightTargetMps(void)
{
    return wheel_target_r_mps;
}

uint8_t Track_IsTurnExitActive(void)
{
    return post_turn_ramp_active;
}

void Track_PrintLastTurn(void)
{
    if (last_turn_valid == 0u) {
        return;
    }
    Link_Printf("[LT] arc=%dcm yawI=%d yaw:%d t=%d.%01ds\r\n",
                (int)last_turn_arc_cm,
                (int)last_turn_yaw_deg,
                (int)turn_target_deg,
                (int)last_turn_time_s,
                (int)((last_turn_time_s - (float)(int)last_turn_time_s) *
                      10.0f + 0.5f));
}
