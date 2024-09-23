//定义ADC通道与引脚，其定义在zf_adc.h文件中 √
#define CHANNEL_NUMBER          (5)
#define ADC_CHANNEL1            (ADC1_CH3_B14)
#define ADC_CHANNEL2            (ADC1_CH4_B15)
#define ADC_CHANNEL3            (ADC1_CH10_B21)
#define ADC_CHANNEL4            (ADC1_CH12_B23)
#define ADC_CHANNEL5            (ADC1_CH5_B16)
uint8 channel_index = 0;
adc_channel_enum adc_channel[CHANNEL_NUMBER] = 
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4, ADC_CHANNEL5
};
uint16 adc_buffer[CHANNEL_NUMBER];

//拨码开关 Switch
#define SWITCH1             (C27)
#define SWITCH2             (C26)
//按键 KEYS 定义在zf_device_key中的KEY1~KEY4
//原始版本的V3.5.5中 key.c和key.h有问题 本工程内的已做修正

//霍尔检测 Hall Detector
#define HALL_PIN            (D4)
//指示器 Indicators
#define BEEP                (B11)
#define LED1                (B9)

//电机 Motors
#define MOTOR1_PWM1                 (PWM1_MODULE0_CHA_D12) // TX4- LED2
#define MOTOR1_PWM2                 (PWM1_MODULE0_CHB_D13) // TX4+ LED4
#define MOTOR2_PWM1                 (PWM1_MODULE1_CHA_D14) // TX5- LED3
#define MOTOR2_PWM2                 (PWM1_MODULE1_CHB_D15) // TX5+ LED5
#define MOTOR_EN                    (B19)
#define MAX_DUTY                    (50)                                                // 最大 MAX_DUTY% 占空比
int8 duty = 0;  //电机信号占空比
bool dir = true;

//编码器（正交） Encoders QUAD
#define ENCODER_1                       (QTIMER1_ENCODER1)
#define ENCODER_1_A                     (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                     (QTIMER1_ENCODER1_CH2_C1)                                 
#define ENCODER_2                       (QTIMER1_ENCODER2)
#define ENCODER_2_A                     (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                     (QTIMER1_ENCODER2_CH2_C24)
//编码器（带方向） Encoders DIR
#define ENCODER_1                       (QTIMER1_ENCODER1)
#define ENCODER_1_LSB                   (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_DIR                   (QTIMER1_ENCODER1_CH2_C1)                                      
#define ENCODER_2                       (QTIMER1_ENCODER2)
#define ENCODER_2_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_DIR                   (QTIMER1_ENCODER2_CH2_C24)
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;


//舵机 Servo
// ------------------ 舵机占空比计算方式 ------------------
// 舵机角度由高电平时间控制
// 0-180 度 = 0.5ms-2.5ms （回中值为90度 1.5ms) 实际可用范围约为0.9ms-2.1ms
// 那么不同频率下的占空比计算方式就是 PWM_DUTY=PWM_DUTY_MAX*（所需角度的ms数/一个周期的ms数)
// 频率不建议超过
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_PWM                (PWM4_MODULE2_CHA_C30)                          // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ                (300)                                           // 定义主板上舵机频率  请务必注意范围 50-300
#define SERVO_MOTOR_L_MAX               (2100)                                           // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX               (4600)                                           // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_MID                (3350)    
#define SPEED_MAX                      (3000)
#define SPEED                           (800)
#define SERVO_MOTOR_DUTY(x)             ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)                               // PWM不建议超过此频率范围
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif
float servo_motor_duty = 90.0;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态

//定时器中断 PITs
#define MAJOR_PIT_CH                  (PIT_CH0)                                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用

