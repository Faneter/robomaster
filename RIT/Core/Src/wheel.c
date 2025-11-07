#include "wheel.h"
#include "tim.h"
#include "pid.h"
#include "math.h"

uint64_t wheel_timestamp;
uint64_t WHEEL_PULSE[4]   = {0, 0, 0, 0};
float wheel_target_rpm[4] = {0, 0, 0, 0};
PID_PWM_TypeDef wheel_pid[4];
float wheel_actual_rpm[4] = {0, 0, 0, 0};
float max_speed           = 1.0f;

void Wheel_Init(void)
{
    // PWM定时器启动
    HAL_TIM_PWM_Start(WHEEL_LF_PWM_TIM, WHEEL_LF_PWM_CHANNEL);
    HAL_TIM_PWM_Start(WHEEL_RF_PWM_TIM, WHEEL_RF_PWM_CHANNEL);
    HAL_TIM_PWM_Start(WHEEL_LB_PWM_TIM, WHEEL_LB_PWM_CHANNEL);
    HAL_TIM_PWM_Start(WHEEL_RB_PWM_TIM, WHEEL_RB_PWM_CHANNEL);

    // 设置状态
    SetWheelDirection(1, STOP);
    SetWheelDirection(2, STOP);
    SetWheelDirection(3, STOP);
    SetWheelDirection(4, STOP);

    // 设置初速
    SetWheelTargetRPM(1, 0.0f);
    SetWheelTargetRPM(2, 0.0f);
    SetWheelTargetRPM(3, 0.0f);
    SetWheelTargetRPM(4, 0.0f);

    PID_Init(&wheel_pid[0], 0.2f, 0.05f, 0.005f, 90.0f, 0.0f);
    PID_Init(&wheel_pid[1], 0.2f, 0.05f, 0.005f, 90.0f, 0.0f);
    PID_Init(&wheel_pid[2], 0.2f, 0.05f, 0.005f, 90.0f, 0.0f);
    PID_Init(&wheel_pid[3], 0.2f, 0.05f, 0.005f, 90.0f, 0.0f);

    wheel_timestamp = timestamp;
}

float GetWheelActualRPM(uint8_t wheel)
{
    return wheel_actual_rpm[wheel - 1];
}

void SetWheelTargetRPM(uint8_t wheel, float rpm)
{
    wheel_target_rpm[wheel - 1] = rpm;
}

void UpdateWheelRPM(uint64_t ts)
{
    uint64_t ms = ts - wheel_timestamp;

    WHEEL_PULSE[3] = __HAL_TIM_GET_COUNTER(&htim4);
    TIM4->CNT      = 0;

    for (int i = 0; i < 4; i++) {
        wheel_actual_rpm[i] = WHEEL_PULSE[i] * 60 * 1000 / 1320 / ms;
        WHEEL_PULSE[i]      = 0;
    }
    wheel_timestamp = ts;

    float pwm1 = PID_Calculate(&wheel_pid[0], wheel_target_rpm[0], wheel_actual_rpm[0]);
    __HAL_TIM_SET_COMPARE(WHEEL_LF_PWM_TIM, WHEEL_LF_PWM_CHANNEL, (uint16_t)(100 * pwm1));
    float pwm2 = PID_Calculate(&wheel_pid[1], wheel_target_rpm[1], wheel_actual_rpm[1]);
    __HAL_TIM_SET_COMPARE(WHEEL_RF_PWM_TIM, WHEEL_RF_PWM_CHANNEL, (uint16_t)(100 * pwm2));
    float pwm3 = 100 * PID_Calculate(&wheel_pid[2], wheel_target_rpm[2], wheel_actual_rpm[2]);
    __HAL_TIM_SET_COMPARE(WHEEL_LB_PWM_TIM, WHEEL_LB_PWM_CHANNEL, (uint16_t)(100 * pwm3));
    float pwm4 = PID_Calculate(&wheel_pid[3], wheel_target_rpm[3], wheel_actual_rpm[3]);
    __HAL_TIM_SET_COMPARE(WHEEL_RB_PWM_TIM, WHEEL_RB_PWM_CHANNEL, (uint16_t)(100 * pwm4));
}

void SetWheelDirection(uint8_t wheel, uint8_t direction)
{
    switch (wheel) {
        case 1:
            switch (direction) {
                case STOP:
                    HAL_GPIO_WritePin(WHEEL_LF_IN1_GPIO, WHEEL_LF_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_LF_IN2_GPIO, WHEEL_LF_IN2, RESET);
                    break;
                case CLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_LF_IN1_GPIO, WHEEL_LF_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_LF_IN2_GPIO, WHEEL_LF_IN2, SET);
                    break;
                case COUNTERCLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_LF_IN1_GPIO, WHEEL_LF_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_LF_IN2_GPIO, WHEEL_LF_IN2, RESET);
                    break;
                case PLUSE:
                    HAL_GPIO_WritePin(WHEEL_LF_IN1_GPIO, WHEEL_LF_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_LF_IN2_GPIO, WHEEL_LF_IN2, SET);
                    break;
                default:
                    break;
            }
            break;
        case 2:
            switch (direction) {
                case STOP:
                    HAL_GPIO_WritePin(WHEEL_RF_IN1_GPIO, WHEEL_RF_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_RF_IN2_GPIO, WHEEL_RF_IN2, RESET);
                    break;
                case CLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_RF_IN1_GPIO, WHEEL_RF_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_RF_IN2_GPIO, WHEEL_RF_IN2, SET);
                    break;
                case COUNTERCLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_RF_IN1_GPIO, WHEEL_RF_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_RF_IN2_GPIO, WHEEL_RF_IN2, RESET);
                    break;
                case PLUSE:
                    HAL_GPIO_WritePin(WHEEL_RF_IN1_GPIO, WHEEL_RF_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_RF_IN2_GPIO, WHEEL_RF_IN2, SET);
                    break;
                default:
                    break;
            }
            break;
        case 3:
            switch (direction) {
                case STOP:
                    HAL_GPIO_WritePin(WHEEL_LB_IN1_GPIO, WHEEL_LB_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_LB_IN2_GPIO, WHEEL_LB_IN2, RESET);
                    break;
                case CLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_LB_IN1_GPIO, WHEEL_LB_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_LB_IN2_GPIO, WHEEL_LB_IN2, SET);
                    break;
                case COUNTERCLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_LB_IN1_GPIO, WHEEL_LB_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_LB_IN2_GPIO, WHEEL_LB_IN2, RESET);
                    break;
                case PLUSE:
                    HAL_GPIO_WritePin(WHEEL_LB_IN1_GPIO, WHEEL_LB_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_LB_IN2_GPIO, WHEEL_LB_IN2, SET);
                    break;
                default:
                    break;
            }
            break;
        case 4:
            switch (direction) {
                case STOP:
                    HAL_GPIO_WritePin(WHEEL_RB_IN1_GPIO, WHEEL_RB_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_RB_IN2_GPIO, WHEEL_RB_IN2, RESET);
                    break;
                case CLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_RB_IN1_GPIO, WHEEL_RB_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_RB_IN2_GPIO, WHEEL_RB_IN2, SET);
                    break;
                case COUNTERCLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_RB_IN1_GPIO, WHEEL_RB_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_RB_IN2_GPIO, WHEEL_RB_IN2, RESET);
                    break;
                case PLUSE:
                    HAL_GPIO_WritePin(WHEEL_RB_IN1_GPIO, WHEEL_RB_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_RB_IN2_GPIO, WHEEL_RB_IN2, SET);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

/**
 * @brief VA轮 = Vx+Vy-Vz*(H/2+W/2)
 *        VB轮 = Vx-Vy-Vz*(H/2+W/2)
 *        VC轮 = Vx+Vy+Vz*(H/2+W/2)
 *        VD轮 = Vx-Vy+Vz*(H/2+W/2)
 *        参数说明：
 *        VABCD轮-> 麦轮A、B、C、D 的线速度，单位m/s。
 *        W-> 轮距，机器人左右麦轮的距离，单位：m。
 *        H-> 轴距，机器人前后麦轮的距离，单位：m。
 *
 * @param Vx 机器人前后移动速度，前进为正，单位：m/s。
 * @param Vy 机器人左右移动速度，左移为正，单位：m/s。
 * @param Vz 机器人绕 O 点旋转速度，逆时针为正，单位：rad/s
 */
void CarMove(float Vx, float Vy, float Vz)
{
    float Va = Vx + Vy - Vz * T;
    float Vb = Vx - Vy - Vz * T;
    float Vc = Vx + Vy + Vz * T;
    float Vd = Vx - Vy + Vz * T;

    if (Va < 0) {
        SetWheelDirection(3, CLOCKWISE);
    } else if (Va > 0) {
        SetWheelDirection(3, COUNTERCLOCKWISE);
    } else {
        SetWheelDirection(3, STOP);
    }
    SetWheelTargetRPM(3, fabsf(Va) * 60 / (RADIUS * 2 * M_PI));

    if (Vb < 0) {
        SetWheelDirection(1, CLOCKWISE);
    } else if (Vb > 0) {
        SetWheelDirection(1, COUNTERCLOCKWISE);
    } else {
        SetWheelDirection(1, STOP);
    }
    SetWheelTargetRPM(1, fabsf(Vb) / (RADIUS * 2 * M_PI) * 60);

    if (Vc < 0) {
        SetWheelDirection(2, CLOCKWISE);
    } else if (Vc > 0) {
        SetWheelDirection(2, COUNTERCLOCKWISE);
    } else {
        SetWheelDirection(2, STOP);
    }
    SetWheelTargetRPM(2, fabsf(Vc) / (RADIUS * 2 * M_PI) * 60);

    if (Vd < 0) {
        SetWheelDirection(4, CLOCKWISE);
    } else if (Vd > 0) {
        SetWheelDirection(4, COUNTERCLOCKWISE);
    } else {
        SetWheelDirection(4, STOP);
    }
    SetWheelTargetRPM(4, fabsf(Vd) / (RADIUS * 2 * M_PI) * 60);
}
