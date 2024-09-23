/*********************************************************************************************************************
* SEU_Smartcar_EM_Demo V2.0 Arthur CJP
* SEU_Smartcar_EM_Demo V1.0 Arthur YCX
* Based on Seekfree RT1064DVL6A Opensource Library V3.5.5
* Update log: Changed to V3.5.5 Library, included MOTOR and SERVO tests
* Warning: 在使用此例程前 请确保电机/舵机未堵转 否则可能烧电机
           烧录时，请断开强电；
           请确保芯片先有电引脚后有电，否则触发保护
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "pindef.h"

volatile uint8 i;
volatile uint8 count_led=0;
uint8 x,y;


    
      int numberindex[5]={0,1,2,3,4};
      int16 s=0;
    
uint8 ParkFlag = 0;    
uint8 garageFlag = 0;    
float Speed_p = 2.0;                                                           //速度控制P
float Speed_i = 0.00;                                                           //* 速度控制I *
float Speed_d = 0.01;                                                          //速度控制D
float Speed_err_pp = 0, Speed_err_p = 0, Speed_err = 0;  
float SpeedI=0;
//速度偏差
//float Speed_err_sum = 0;
float RealSpeed = 0;
float Speed_out = 0;                                                            //电机PWM控制输出  0~1000  0~100%
float Speed_set = 0;
int SpeedCount=0;

float Dir_p = 1.0;                                                             //* 默认方向控制P *  (响应速度，参考角度，最大打角
float Dir_d = 0.5;//0.2865                                                           //方向控制D
float Dir_i = 0.0;
float Dir_err = 0.0;
float Dir_err_p = 0.0;
float Dir_err_sum = 0.0;
int Dir_out = 0;


int roundtimes=0;
int roundflag=0;
int lastroundflag=0;
int lasts=0;
int delayms=0;
int parkdelay=0;

void major_init(void)
{
    // ENCODE
    encoder_quad_init(ENCODER_1, ENCODER_1_LSB, ENCODER_1_DIR);                 // 初始化编码器模块与引脚 正交（方向）解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_LSB, ENCODER_2_DIR);                 // 初始化编码器模块与引脚 正交（方向）解码编码器模式
    // ADC
    adc_init(ADC_CHANNEL1, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL2, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL3, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL4, ADC_12BIT);  
    adc_init(ADC_CHANNEL5, ADC_12BIT); 
                                        // 初始化对应 ADC 通道为对应精度
    // SWITCH
    gpio_init(SWITCH1, GPI, 0,  GPI_PULL_UP);                                   // 初始化拨码开关 
    gpio_init(SWITCH2, GPI, 0,  GPI_PULL_UP);                                   // 初始化拨码开关 
    // INDICATOR
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化指示灯（核心板蓝灯）
    gpio_init(BEEP, GPO, GPIO_LOW,  GPO_PUSH_PULL);                             // 初始化蜂鸣器
    // MOTOR 
    gpio_init(MOTOR_EN, GPO, 1, GPO_PUSH_PULL);                                 // 初始化电机使能
    pwm_init(MOTOR1_PWM1, 17000, 0);	                                        // 初始化电机PWM 
    pwm_init(MOTOR1_PWM2, 17000, 0);					        // 初始化电机PWM
    pwm_init(MOTOR2_PWM1, 17000, 0);					        // 初始化电机PWM
    pwm_init(MOTOR2_PWM2, 17000, 0);	                                        // 初始化电机PWM
    // SERVO
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, SERVO_MOTOR_MID);          // 初始化舵机（回中）
//    ips200_init(IPS200_TYPE_PARALLEL8);
    tft180_init();
    gpio_init(HALL_PIN, GPI, GPIO_HIGH, GPI_PULL_DOWN);
    key_init(10);
}

 int numbers[5]={0};

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);                                              // 不可删除
    debug_init();                                                               // 调试端口初始化
    system_delay_ms(300);                                                       // 等待主板其他外设上电完成
    
    major_init();                                                               // 用户自定义初始化函数
   
    // 以上为初始化 以下为body
    
    // BEEP两下 表示初始化好了
    gpio_set_level(BEEP, GPIO_HIGH);
    system_delay_ms(300);
    gpio_set_level(BEEP, GPIO_LOW);
     system_delay_ms(150);
    gpio_set_level(BEEP, GPIO_HIGH);
    system_delay_ms(300);
    gpio_set_level(BEEP, GPIO_LOW);
    
                        
    
    // 舵机转一下 转角请自行做修正 可能中点不是1.5ms 转向也有可能不是90度/1ms
    pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_MID );
   // return;
      // 电机转起来 正反转请自行测试（例如 PWM2给3000 PWM1给0）
    pwm_set_duty(MOTOR1_PWM1, 0);	                                        
    pwm_set_duty(MOTOR1_PWM2, 0);					        
          system_delay_ms(500);         
//    ips200_show_string(0, 0, "seekfree");
          pwm_set_duty(MOTOR1_PWM1, 3000);	                                        
          pwm_set_duty(MOTOR1_PWM2, 0);					        
          system_delay_ms(500);
          pwm_set_duty(MOTOR1_PWM1, 0);	                                        
          pwm_set_duty(MOTOR1_PWM2, 0);
          pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_L_MAX+250);
          pwm_set_duty(MOTOR1_PWM1, 3000);	                                        
          pwm_set_duty(MOTOR1_PWM2, 0);					        
          system_delay_ms(450);
          pwm_set_duty(MOTOR1_PWM1, 0);	                                        
          pwm_set_duty(MOTOR1_PWM2, 0);
          // PIT
    pit_ms_init(MAJOR_PIT_CH, 1);                                             // 初始化主pit 修改PIT号需修改对应的isr.c内的函数调用!!!
    NVIC_SetPriority(PIT_IRQn, 2);
    NVIC_SetPriority(CSI_IRQn, 0);
    //开启总中断后进入死循环
    interrupt_global_enable(0);
    while(1)
    {
       // 读取数据采集后输出到串口 当然也可使用watch观察
       // 串口用法：使用蓝牙或USB串口连接UART1
       // TX-RX，RX-TX，GND-GND，注意先给板子上电再插串口 否则会因上电前引脚有电压而触发芯片自动保护
       //（这里的“UART1”定义在zf_common_debug.c/h中，可修改）
       // 波特率115200 此时由于debug文件的定义 printf将打印到串口 串口调试助手能收到信息
       // printf("ENC1=%d.", encoder_data_1);                                     // 输出编码器计数信息
       // printf("ENC2=%d.", encoder_data_2);                                     // 输出编码器计数信息
       // printf("ADC0=%d.", adc_buffer[0]);                                      // 输出ADC采样信息
       // printf("ADC1=%d.", adc_buffer[1]);                                      // 输出ADC采样信息
       // printf("ADC2=%d.", adc_buffer[2]);                                      // 输出ADC采样信息
       // printf("ADC3=%d.\r\n", adc_buffer[3]);                                  // 输出ADC采样信息
        tft180_show_int(0, 0, encoder_data_1, 4);   
        tft180_show_int(0, 20, ParkFlag, 4);  
        tft180_show_int(0, 40, Speed_set, 4); 
        tft180_show_int(0, 60,garageFlag,4);
        tft180_show_int(0, 80, Speed_out, 4);
       // tft180_show_int(0, 80, roundtimes, 4);
        //tft180_show_int(0, 100, roundflag, 4);
        //tft180_show_int(0, 120, lastroundflag, 4);
        
         tft180_show_int(40, 120, delayms, 4);
             
        tft180_show_uint(50,  0, adc_buffer[0],  4); 
        tft180_show_uint(40, 20, adc_buffer[1],  4); 
        tft180_show_uint(40, 40, adc_buffer[2],  4); 
        tft180_show_uint(40, 60, adc_buffer[3],  4); 
        tft180_show_uint(40, 80, adc_buffer[4],  4); 
        tft180_show_int(80, 80, Dir_out ,  4); 
        tft180_show_int(80, 100, s ,  4); 
        
          //tft180_show_string(0, 0, "seekfree");
        // FROM: Seekfree Example: Switch(simplified)
        // 注意这个样例的原始代码里有用到zf_device_key.c/h 这两个文件有bug 
        // 本工程内的已做修正 如要使用可参考修正
        
        
        x=gpio_get_level(SWITCH1);
        y=gpio_get_level(SWITCH2);
        if(!gpio_get_level(SWITCH1) || !gpio_get_level(SWITCH2))                // SWITCH1/SWITCH2 ON
        {
          ParkFlag=1;
            if(25 > count_led)
                gpio_set_level(LED1, GPIO_LOW);                                 // LED1 亮
            else
                gpio_set_level(LED1, GPIO_HIGH);                                // LED1 灭
        }
        else
            gpio_set_level(LED1, GPIO_HIGH);                                    // LED1 灭                         
        count_led = ((count_led != 100) ? (count_led + 1) : (1));
        
        /*if(garageFlag==1)
        {
          Speed_set=0;
          Dir_out=SERVO_MOTOR_L_MAX;
          Speed_set=-3000;					        
          Speed_set=0;
        }*/
    }
}


void bubbleSort(int arr[],int arr1[] ,int n) {
    for (int i = 0; i < n-1; i++) {
        for (int j = 0; j < n-i-1; j++) {
            if (arr[j] < arr[j+1]) {
                
                int temp = arr[j];
    arr[j] = arr[j+1];
    arr[j+1] = temp;
                
                  int temp1 = arr1[j];
    arr1[j] = arr1[j+1];
    arr1[j+1] = temp1;
            }
        }
    }
}

int zone(int numberindex[],int numbers[])
{
  lasts=s;
  lastroundflag=roundflag;
  if(adc_buffer[2]+adc_buffer[3]+adc_buffer[4]>4000)
  {
    roundflag=0;
    if(roundtimes%2 == 1 )//&&lastroundflag==1)
    {
      //gpio_set_level(BEEP, GPIO_HIGH);
      //system_delay_ms(300);
      //gpio_set_level(BEEP, GPIO_LOW);
      //system_delay_ms(150);
      //gpio_set_level(BEEP, GPIO_HIGH);
      //system_delay_ms(300);
      //gpio_set_level(BEEP, GPIO_LOW);
      s=8;
    }
    if(roundtimes%2 ==0 && s!=8 && lastroundflag==1)
    {
      gpio_set_level(BEEP, GPIO_LOW);
      s=9;
    }
  }
  else
  {
    if(numbers[0]>50)
    {
      if(s!=8)
      {
        roundflag=1;
        if(lastroundflag==0)
          roundtimes++;
        if(numberindex[0]== 0)
        {
          if(numberindex[1]== 1)
          {
            s=0;
          }
          
        }
        else if(numberindex[0]== 1)
        {
          if(numberindex[1]== 0)
          {
            if(numberindex[2]== 2)
            {
              s=1;
            }
          }
          else if(numberindex[1]== 2)
          {
            if(numberindex[2]== 0)
            {
              s=2;
            }
          }
        }
        
        else if(numberindex[0]== 2)
        {
          if(numberindex[1]== 1)
          {
            if(numberindex[2]== 3)
            {
              s=3;
            }
          }
          else if(numberindex[1]== 3)
          {
            if(numberindex[2]== 1)
            {
              s=4;
            }
          }
        }
        
        else if(numberindex[0]== 3)
        {
          if(numberindex[1]== 2)
          {
            if(numberindex[2]== 4)
            {
              s=5;
            }
          }
          else if(numberindex[1]== 4)
          {
            if(numberindex[2]== 2)
            {
              s=6;
            }
          }
        }
        
        else if(numberindex[0]== 4)
        {
          if(numberindex[1]== 3)
          {
            s=7;
          }
          
        }
      }
    }
    else
    {
      s=lasts;
    }
  }
  return s;
}

float kvalue(int a,int b)
{
  return (adc_buffer[a]-adc_buffer[b]+0.001)*1.0/(adc_buffer[a]+adc_buffer[b])*1.0;
}


void SpeedControl(void)
{
    //获取速度信息
    //
  SpeedCount++;
  if((SpeedCount%10==0)||(SpeedI==1))
  {
    if(ParkFlag==1)
    {
      Speed_set=0;
    }
    
    RealSpeed =-(float)encoder_data_1 * 25;
    
    
    Speed_err_pp = Speed_err_p;
    Speed_err_p = Speed_err;
    Speed_err = Speed_set - RealSpeed;
    
    Speed_out = Speed_out + Speed_p * (Speed_err-Speed_err_p)+Speed_i * Speed_err + Speed_d * (Speed_err - 2 * Speed_err_p + Speed_err_pp);
    if(Speed_out>SPEED_MAX) Speed_out=SPEED_MAX;
    if(Speed_out<-SPEED_MAX) Speed_out=-SPEED_MAX;
    if(Speed_out>=0)
    {
    pwm_set_duty(MOTOR1_PWM1,(uint32)Speed_out);
    pwm_set_duty(MOTOR1_PWM2, 0);
    }
    else
    {
    pwm_set_duty(MOTOR1_PWM1,0);
    pwm_set_duty(MOTOR1_PWM2, (uint32)(-Speed_out));
    }
  }
}

void DirControl(void)
{
    Dir_err_p = Dir_err;
    Dir_err = Dir_out-SERVO_MOTOR_MID;
    Dir_err_sum = Dir_err_sum + Dir_err;
    Dir_out = Dir_p * Dir_err + Dir_i * Dir_err_sum + Dir_d * (Dir_err_p - Dir_err)+SERVO_MOTOR_MID;      //位置式 PID 控制
    if(Dir_out>SERVO_MOTOR_R_MAX) Dir_out=SERVO_MOTOR_R_MAX;
    if(Dir_out<SERVO_MOTOR_L_MAX) Dir_out=SERVO_MOTOR_L_MAX;
    pwm_set_duty(SERVO_MOTOR_PWM, Dir_out); 
}


void major_pit_handler (void)                                                  // 总中断处理函数，在isr.c中调用
{
    encoder_data_1 = encoder_get_count(ENCODER_1);                              // 获取编码器计数 正反转请自行测试和处理
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

   
    
    for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index ++)    //获取ADC采样值 在悬空时大概为1000-2000 可接入3V3/GND测试
        adc_buffer[channel_index]=adc_mean_filter_convert(adc_channel[channel_index],5);
    
     adc_buffer[3]-=20;
    if(adc_buffer[3]<0) adc_buffer[3]=0;
    if(adc_buffer[3]>5000) adc_buffer[3]=3;

     adc_buffer[4]-=20;
    if(adc_buffer[4]<0) adc_buffer[4]=0;
    if(adc_buffer[4]>5000) adc_buffer[4]=3;

   
    numbers[0]=adc_buffer[0];
    numbers[1]=adc_buffer[1];
    numbers[2]=adc_buffer[2];
    numbers[3]=adc_buffer[3];
    numbers[4]=adc_buffer[4];
    numberindex[0]=0;
    numberindex[1]=1;
    numberindex[2]=2;
    numberindex[3]=3;
    numberindex[4]=4;
    
     bubbleSort(numbers,numberindex, 5);
     
     
     
     if(ParkFlag==0 && garageFlag==0  )
     {
       //if(numbers[0]>50)
       {
         switch (zone(numberindex,numbers))
         {
         case 0:
           Dir_out=SERVO_MOTOR_L_MAX;
           //SpeedI++;
           //if(SpeedI==1)
           {
             //gpio_set_level(BEEP, GPIO_HIGH);
             Speed_set=-SPEED_MAX;
    
           }
           //else 
           {Speed_set= 1.4*SPEED;}
           
           break;
         case 1:
           Dir_out=(1.0/(kvalue(1,0)/kvalue(0,2)+1.0))*(SERVO_MOTOR_L_MAX-(1.0*SERVO_MOTOR_L_MAX/2.0+SERVO_MOTOR_MID*1.0/2.0))+(1.0*SERVO_MOTOR_L_MAX/2.0+SERVO_MOTOR_MID*1.0/2.0);
          
           Speed_set =1.4*SPEED;
           break;
         case 2:
           Dir_out=(1.0/(kvalue(2,0)/kvalue(1,2)+1.0))*((1.0/2.0*SERVO_MOTOR_L_MAX+SERVO_MOTOR_MID*1.0/2.0)-(1.0*SERVO_MOTOR_L_MAX/6.0+SERVO_MOTOR_MID*5.0/6.0))+(1.0*SERVO_MOTOR_L_MAX/6.0+SERVO_MOTOR_MID*5.0/6.0);
          
           Speed_set = 1.5*SPEED;
           break;
         case 3:
           Dir_out=(1.0/(kvalue(2,1)/kvalue(1,3)+1.0))*((1.0/6.0*SERVO_MOTOR_L_MAX+SERVO_MOTOR_MID*5.0/6.0)-SERVO_MOTOR_MID)+SERVO_MOTOR_MID;
           
           
           Speed_set = 1.6*SPEED;
           break;
         case 4:
           Dir_out=(1.0/(kvalue(3,1)/kvalue(2,3)+1.0))*(-(1.0/6.0*SERVO_MOTOR_R_MAX+SERVO_MOTOR_MID*5.0/6.0)+SERVO_MOTOR_MID)+(1.0/6.0*SERVO_MOTOR_R_MAX+SERVO_MOTOR_MID*5.0/6.0);
           
           Speed_set = 1.6*SPEED;
           break;
         case 5:
           Dir_out=(1.0/(kvalue(3,2)/kvalue(2,4)+1.0))*(-(1.0/2.0*SERVO_MOTOR_R_MAX+SERVO_MOTOR_MID*1.0/2.0)+(1.0*SERVO_MOTOR_R_MAX/6.0+SERVO_MOTOR_MID*5.0/6.0))+(1.0*SERVO_MOTOR_R_MAX/2.0+SERVO_MOTOR_MID*1.0/2.0);
           
           Speed_set = 1.5*SPEED;
           break;
         case 6:
           Dir_out=(1.0/(kvalue(4,2)/kvalue(3,4)+1.0))*(-SERVO_MOTOR_R_MAX+(1.0*SERVO_MOTOR_R_MAX/2.0+SERVO_MOTOR_MID*1.0/2.0))+SERVO_MOTOR_R_MAX;
           
           Speed_set = 1.4*SPEED;
           break;
         case 7:
           Dir_out=SERVO_MOTOR_R_MAX;
           //SpeedI++;
           //if(SpeedI==1)
           //{Speed_set =-SPEED_MAX;}
          // else
           {
             Speed_set =1.4*SPEED;
           }
           break;
         case 8:
           delayms-=encoder_data_1;
           
           if(delayms> 2000 && delayms <4500 )
           {
             Dir_out=SERVO_MOTOR_L_MAX;
           }
           else if(delayms> 4500)
           {
             s=0;
           }
           else if(delayms<2000)
           {
             Dir_out=SERVO_MOTOR_MID;
           }
           
           Speed_set =1.4*SPEED;
           
           break;
         case 9:
           Dir_out=SERVO_MOTOR_MID;
           delayms=0;
           Speed_set =1.4*SPEED;
           break;
         }
       }
       /*else
       {
         if(adc_buffer[0]>adc_buffer[4])
         {
           Dir_out=SERVO_MOTOR_L_MAX;
         }
         else if(adc_buffer[0]<adc_buffer[4])
         {
           Dir_out=SERVO_MOTOR_R_MAX;
         }
         
       }*/
     
}
     
     if(numbers[0]<10) ParkFlag=1;
     
     if(gpio_get_level(HALL_PIN)==0)//需要测试高低电平,先默认是低电平
    {
      //ParkFlag=1;//发车时会有霍尔检测吗
      garageFlag=1;
    }
    if(garageFlag==1)
        {
 
          parkdelay+=abs(encoder_data_1);
          if(parkdelay>1750 && parkdelay< 4300)
          {
            Dir_out=SERVO_MOTOR_L_MAX;
            Speed_set=-2000;
          
          }
          else
          {
            Dir_out=SERVO_MOTOR_MID;
            Speed_set=0;
          }
        }
       //pwm_set_duty(SERVO_MOTOR_PWM, Dir_out); 
       DirControl();
       SpeedControl();
       
}





