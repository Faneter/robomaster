#include "servo.h"

Servo_Control_Data servo_data;

void Servo_Init(void)
{
    // PWM定时器启动
    HAL_TIM_PWM_Start(SERVO_1_PWM_TIM, SERVO_1_PWM_CHANNEL);
    HAL_TIM_PWM_Start(SERVO_2_PWM_TIM, SERVO_2_PWM_CHANNEL);
    HAL_TIM_PWM_Start(SERVO_3_PWM_TIM, SERVO_3_PWM_CHANNEL);
    HAL_TIM_PWM_Start(SERVO_4_PWM_TIM, SERVO_4_PWM_CHANNEL);

    // 舵机初始化
    servo_data.servo1 = 0;
    servo_data.servo2 = 0;
    servo_data.servo3 = 0;
    servo_data.servo4 = 0;

    UpdateServoAngle();
}

void SetServoAngle(uint8_t servo, uint8_t angle)
{
    if (angle > 180) {
        angle = 180;
    }
    switch (servo) {
        case 1:
            __HAL_TIM_SET_COMPARE(SERVO_1_PWM_TIM, SERVO_1_PWM_CHANNEL, angle + 45);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(SERVO_2_PWM_TIM, SERVO_2_PWM_CHANNEL, angle + 45);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(SERVO_3_PWM_TIM, SERVO_3_PWM_CHANNEL, angle + 45);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(SERVO_4_PWM_TIM, SERVO_4_PWM_CHANNEL, angle + 45);
            break;
        default:
            break;
    }
}

void UpdateServoAngle()
{
    SetServoAngle(1, servo_data.servo1);
    SetServoAngle(2, servo_data.servo2);
    SetServoAngle(3, servo_data.servo3);
    SetServoAngle(4, servo_data.servo4);
}
