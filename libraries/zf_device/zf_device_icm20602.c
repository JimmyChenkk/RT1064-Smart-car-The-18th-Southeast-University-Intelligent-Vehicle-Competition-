/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          zf_device_icm20602
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   // Ӳ�� SPI ����
*                   SCL/SPC             �鿴 zf_device_icm20602.h �� ICM20602_SPC_PIN �궨��
*                   SDA/SDI             �鿴 zf_device_icm20602.h �� ICM20602_SDI_PIN �궨��
*                   SA0/SDO             �鿴 zf_device_icm20602.h �� ICM20602_SDO_PIN �궨��
*                   CS                  �鿴 zf_device_icm20602.h �� ICM20602_CS_PIN �궨��
*                   VCC                 3.3V��Դ
*                   GND                 ��Դ��
*                   ������������
*
*                   // ���� IIC ����
*                   SCL/SPC             �鿴 zf_device_icm20602.h �� ICM20602_SCL_PIN �궨��
*                   SDA/SDI             �鿴 zf_device_icm20602.h �� ICM20602_SDA_PIN �궨��
*                   VCC                 3.3V��Դ
*                   GND                 ��Դ��
*                   ������������
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_soft_iic.h"

#include "zf_device_icm20602.h"

int16 icm20602_gyro_x = 0, icm20602_gyro_y = 0, icm20602_gyro_z = 0;            // ��������������      gyro (������)
int16 icm20602_acc_x = 0, icm20602_acc_y = 0, icm20602_acc_z = 0;               // ������ٶȼ�����    acc (accelerometer ���ٶȼ�)
float icm20602_transition_factor[2] = {4096, 16.4};

#if ICM20602_USE_SOFT_IIC
static soft_iic_info_struct icm20602_iic_struct;

//-------------------------------------------------------------------------------------------------------------------
// �������     ICM20602 д�Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define icm20602_write_register(reg, data)      (soft_iic_write_8bit_register(&icm20602_iic_struct, (reg), (data)))

//-------------------------------------------------------------------------------------------------------------------
// �������     ICM20602 ���Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ���ز���     uint8           ����
// ʹ��ʾ��     icm20602_read_register(ICM20602_WHO_AM_I);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define icm20602_read_register(reg)             (soft_iic_read_8bit_register(&icm20602_iic_struct, (reg)))

//-------------------------------------------------------------------------------------------------------------------
// �������     ICM20602 ������
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ���ݻ�����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define icm20602_read_registers(reg, data, len) (soft_iic_read_8bit_registers(&icm20602_iic_struct, (reg), (data), (len)))
#else
//-------------------------------------------------------------------------------------------------------------------
// �������     ICM20602 д�Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_write_register (uint8 reg, uint8 data)
{
    ICM20602_CS(0);
    spi_write_8bit_register(ICM20602_SPI, reg | ICM20602_SPI_W, data);
    ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ICM20602 ���Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ���ز���     uint8           ����
// ʹ��ʾ��     icm20602_read_register(ICM20602_WHO_AM_I);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8 icm20602_read_register (uint8 reg)
{
    uint8 data = 0;
    ICM20602_CS(0);
    data = spi_read_8bit_register(ICM20602_SPI, reg | ICM20602_SPI_R);
    ICM20602_CS(1);
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ICM20602 ������
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ���ݻ�����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_read_registers (uint8 reg, uint8 *data, uint32 len)
{
    ICM20602_CS(0);
    spi_read_8bit_registers(ICM20602_SPI, reg | ICM20602_SPI_R, data, len);
    ICM20602_CS(1);
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// �������     ICM20602 �Լ�
// ����˵��     void
// ���ز���     uint8           1-�Լ�ʧ�� 0-�Լ�ɹ�
// ʹ��ʾ��     icm20602_self_check();
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8 icm20602_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;

    while(0x12 != dat)                                                          // �ж� ID �Ƿ���ȷ
    {
        if(ICM20602_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state =  1;
            break;
        }
        dat = icm20602_read_register(ICM20602_WHO_AM_I);
        system_delay_ms(10);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ ICM20602 ���ٶȼ�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     icm20602_get_acc();                                             // ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void icm20602_get_acc (void)
{
    uint8 dat[6];

    icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
    icm20602_acc_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    icm20602_acc_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    icm20602_acc_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡICM20602����������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     icm20602_get_gyro();                                            // ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void icm20602_get_gyro (void)
{
    uint8 dat[6];

    icm20602_read_registers(ICM20602_GYRO_XOUT_H, dat, 6);
    icm20602_gyro_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    icm20602_gyro_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    icm20602_gyro_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�� ICM20602
// ����˵��     void
// ���ز���     uint8           1-��ʼ��ʧ�� 0-��ʼ���ɹ�
// ʹ��ʾ��     icm20602_init();
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
uint8 icm20602_init (void)
{
    uint8 val = 0x0, return_state = 0;
    uint16 timeout_count = 0;

    system_delay_ms(10);                                                        // �ϵ���ʱ

#if ICM20602_USE_SOFT_IIC
    soft_iic_init(&icm20602_iic_struct, ICM20602_DEV_ADDR, ICM20602_SOFT_IIC_DELAY, ICM20602_SCL_PIN, ICM20602_SDA_PIN);
#else
    spi_init(ICM20602_SPI, SPI_MODE0, ICM20602_SPI_SPEED, ICM20602_SPC_PIN, ICM20602_SDI_PIN, ICM20602_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM20602_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
#endif

    do
    {
        if(icm20602_self_check())
        {
            // �������������˶�����Ϣ ������ʾ����λ��������
            // ��ô���� ICM20602 �Լ��������ʱ�˳���
            // ���һ�½�����û������ ���û������ܾ��ǻ���
            zf_log(0, "icm20602 self check error.");
            return_state = 1;
            break;
        }

        icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);                     // ��λ�豸
        system_delay_ms(2);

        do
        {                                                                       // �ȴ���λ�ɹ�
            val = icm20602_read_register(ICM20602_PWR_MGMT_1);
            if(ICM20602_TIMEOUT_COUNT < timeout_count ++)
            {
                // �������������˶�����Ϣ ������ʾ����λ��������
                // ��ô���� ICM20602 �Լ��������ʱ�˳���
                // ���һ�½�����û������ ���û������ܾ��ǻ���
                zf_log(0, "icm20602 reset error.");
                return_state = 1;
                break;
            }
        }while(0x41 != val);
        if(1 == return_state)
        {
            break;
        }

        icm20602_write_register(ICM20602_PWR_MGMT_1,     0x01);                 // ʱ������
        icm20602_write_register(ICM20602_PWR_MGMT_2,     0x00);                 // ���������Ǻͼ��ٶȼ�
        icm20602_write_register(ICM20602_CONFIG,         0x01);                 // 176HZ 1KHZ
        icm20602_write_register(ICM20602_SMPLRT_DIV,     0x07);                 // �������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)

        // ICM20602_ACCEL_CONFIG �Ĵ���
        // ����Ϊ 0x00 ���ٶȼ�����Ϊ ��2  g   ��ȡ���ļ��ٶȼ����ݳ��� 16384  ����ת��Ϊ��������λ������ (g �����������ٶ� ����ѧ���� һ������� g ȡ 9.8 m/s^2 Ϊ��׼ֵ)
        // ����Ϊ 0x08 ���ٶȼ�����Ϊ ��4  g   ��ȡ���ļ��ٶȼ����ݳ��� 8192   ����ת��Ϊ��������λ������ (g �����������ٶ� ����ѧ���� һ������� g ȡ 9.8 m/s^2 Ϊ��׼ֵ)
        // ����Ϊ 0x10 ���ٶȼ�����Ϊ ��8  g   ��ȡ���ļ��ٶȼ����ݳ��� 4096   ����ת��Ϊ��������λ������ (g �����������ٶ� ����ѧ���� һ������� g ȡ 9.8 m/s^2 Ϊ��׼ֵ)
        // ����Ϊ 0x18 ���ٶȼ�����Ϊ ��16 g   ��ȡ���ļ��ٶȼ����ݳ��� 2048   ����ת��Ϊ��������λ������ (g �����������ٶ� ����ѧ���� һ������� g ȡ 9.8 m/s^2 Ϊ��׼ֵ)
        switch(ICM20602_ACC_SAMPLE_DEFAULT)
        {
            default:
            {
                zf_log(0, "ICM20602_ACC_SAMPLE_DEFAULT set error.");
                return_state = 1;
            }break;
            case ICM20602_ACC_SAMPLE_SGN_2G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x00);
                icm20602_transition_factor[0] = 16384;
            }break;
            case ICM20602_ACC_SAMPLE_SGN_4G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x08);
                icm20602_transition_factor[0] = 8192;
            }break;
            case ICM20602_ACC_SAMPLE_SGN_8G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x10);
                icm20602_transition_factor[0] = 4096;
            }break;
            case ICM20602_ACC_SAMPLE_SGN_16G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x18);
                icm20602_transition_factor[0] = 2048;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        // ICM20602_GYRO_CONFIG �Ĵ���
        // ����Ϊ 0x00 ����������Ϊ ��250  dps    ��ȡ�������������ݳ��� 131     ����ת��Ϊ��������λ������ ��λΪ ��/s
        // ����Ϊ 0x08 ����������Ϊ ��500  dps    ��ȡ�������������ݳ��� 65.5    ����ת��Ϊ��������λ������ ��λΪ ��/s
        // ����Ϊ 0x10 ����������Ϊ ��1000 dps    ��ȡ�������������ݳ��� 32.8    ����ת��Ϊ��������λ������ ��λΪ ��/s
        // ����Ϊ 0x18 ����������Ϊ ��2000 dps    ��ȡ�������������ݳ��� 16.4    ����ת��Ϊ��������λ������ ��λΪ ��/s
        switch(ICM20602_GYRO_SAMPLE_DEFAULT)
        {
            default:
            {
                zf_log(0, "ICM20602_GYRO_SAMPLE_DEFAULT set error.");
                return_state = 1;
            }break;
            case ICM20602_GYRO_SAMPLE_SGN_250DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x00);
                icm20602_transition_factor[1] = 131.0;
            }break;
            case ICM20602_GYRO_SAMPLE_SGN_500DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x08);
                icm20602_transition_factor[1] = 65.5;
            }break;
            case ICM20602_GYRO_SAMPLE_SGN_1000DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x10);
                icm20602_transition_factor[1] = 32.8;
            }break;
            case ICM20602_GYRO_SAMPLE_SGN_2000DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x18);
                icm20602_transition_factor[1] = 16.4;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        icm20602_write_register(ICM20602_ACCEL_CONFIG_2, 0x03);                 // Average 4 samples   44.8HZ   //0x23 Average 16 samples
    }while(0);
    return return_state;
}