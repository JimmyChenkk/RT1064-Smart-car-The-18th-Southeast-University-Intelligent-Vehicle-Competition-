//����ADCͨ�������ţ��䶨����zf_adc.h�ļ��� ��
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

//���뿪�� Switch
#define SWITCH1             (C27)
#define SWITCH2             (C26)
//���� KEYS ������zf_device_key�е�KEY1~KEY4
//ԭʼ�汾��V3.5.5�� key.c��key.h������ �������ڵ���������

//������� Hall Detector
#define HALL_PIN            (D4)
//ָʾ�� Indicators
#define BEEP                (B11)
#define LED1                (B9)

//��� Motors
#define MOTOR1_PWM1                 (PWM1_MODULE0_CHA_D12) // TX4- LED2
#define MOTOR1_PWM2                 (PWM1_MODULE0_CHB_D13) // TX4+ LED4
#define MOTOR2_PWM1                 (PWM1_MODULE1_CHA_D14) // TX5- LED3
#define MOTOR2_PWM2                 (PWM1_MODULE1_CHB_D15) // TX5+ LED5
#define MOTOR_EN                    (B19)
#define MAX_DUTY                    (50)                                                // ��� MAX_DUTY% ռ�ձ�
int8 duty = 0;  //����ź�ռ�ձ�
bool dir = true;

//�������������� Encoders QUAD
#define ENCODER_1                       (QTIMER1_ENCODER1)
#define ENCODER_1_A                     (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                     (QTIMER1_ENCODER1_CH2_C1)                                 
#define ENCODER_2                       (QTIMER1_ENCODER2)
#define ENCODER_2_A                     (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                     (QTIMER1_ENCODER2_CH2_C24)
//�������������� Encoders DIR
#define ENCODER_1                       (QTIMER1_ENCODER1)
#define ENCODER_1_LSB                   (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_DIR                   (QTIMER1_ENCODER1_CH2_C1)                                      
#define ENCODER_2                       (QTIMER1_ENCODER2)
#define ENCODER_2_LSB                   (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_DIR                   (QTIMER1_ENCODER2_CH2_C24)
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;


//��� Servo
// ------------------ ���ռ�ձȼ��㷽ʽ ------------------
// ����Ƕ��ɸߵ�ƽʱ�����
// 0-180 �� = 0.5ms-2.5ms ������ֵΪ90�� 1.5ms) ʵ�ʿ��÷�ΧԼΪ0.9ms-2.1ms
// ��ô��ͬƵ���µ�ռ�ձȼ��㷽ʽ���� PWM_DUTY=PWM_DUTY_MAX*������Ƕȵ�ms��/һ�����ڵ�ms��)
// Ƶ�ʲ����鳬��
// ------------------ ���ռ�ձȼ��㷽ʽ ------------------
#define SERVO_MOTOR_PWM                (PWM4_MODULE2_CHA_C30)                          // ���������϶����Ӧ����
#define SERVO_MOTOR_FREQ                (300)                                           // ���������϶��Ƶ��  �����ע�ⷶΧ 50-300
#define SERVO_MOTOR_L_MAX               (2100)                                           // ���������϶�����Χ �Ƕ�
#define SERVO_MOTOR_R_MAX               (4600)                                           // ���������϶�����Χ �Ƕ�
#define SERVO_MOTOR_MID                (3350)    
#define SPEED_MAX                      (3000)
#define SPEED                           (800)
#define SERVO_MOTOR_DUTY(x)             ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)                               // PWM�����鳬����Ƶ�ʷ�Χ
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif
float servo_motor_duty = 90.0;                                                  // ��������Ƕ�
float servo_motor_dir = 1;                                                      // �������״̬

//��ʱ���ж� PITs
#define MAJOR_PIT_CH                  (PIT_CH0)                                 // ʹ�õ������жϱ�� ����޸� ��Ҫͬ����Ӧ�޸������жϱ���� isr.c �еĵ���

