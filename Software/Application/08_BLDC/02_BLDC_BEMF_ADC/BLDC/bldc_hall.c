/*
*******************************************************************************************************
*
* File Name : bldc_hall.c
* Version   : V1.0
* Author    : mzy2364
* brief     : bldc hall driver
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "motor_hardware.h"
#include "bldc_hall.h"
#include "systick.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/*  六步换相 */
static void m1_uhvl(void);
static void m1_uhwl(void);
static void m1_vhwl(void);
static void m1_vhul(void);
static void m1_whul(void);
static void m1_whvl(void);

static uint8_t uemf_edge(uint8_t val);
static uint32_t hallsensor_get_state(void);

/* VARIABLES ----------------------------------------------------------------------------------------*/

_bldc_obj g_bldc_motor = {0};   /* 电机结构体初始值 */

/*  六步换向函数指针数组 */
pctr pfunclist_m1[6] =
{
    &m1_uhwl, &m1_vhul, &m1_vhwl,
    &m1_whvl, &m1_uhvl, &m1_whul
};

uint8_t adc_channel[6] = 
{
    BEMFV_ADC_CHANNEL,BEMFW_ADC_CHANNEL,BEMFU_ADC_CHANNEL,
    BEMFU_ADC_CHANNEL,BEMFW_ADC_CHANNEL,BEMFV_ADC_CHANNEL
};

/* FUNCTION -----------------------------------------------------------------------------------------*/
void bldc_init(void)
{
    g_bldc_motor.dir = CW;
    g_bldc_motor.count_j = 0;
    g_bldc_motor.lock_time = 0;
    g_bldc_motor.pwm_duty = 0;
    g_bldc_motor.speed = 0;
    g_bldc_motor.run_flag = STOP;
    
	g_bldc_motor.speed_pi.kp = WKP;
	g_bldc_motor.speed_pi.ki = WKI;
	g_bldc_motor.speed_pi.kc = WKC;
	g_bldc_motor.speed_pi.out_max = WOUTMAX;
	g_bldc_motor.speed_pi.out_min = MIN_PWM_DUTY;
	
	init_pi(&g_bldc_motor.speed_pi);
}

/**
 * @brief       BLDC控制函数
 * @param       dir :电机方向, Duty:PWM占空比
 * @retval      无
 */
void bldc_ctrl(int32_t dir,float duty)
{
	g_bldc_motor.dir = dir;            /* 方向 */
	g_bldc_motor.pwm_duty = duty;      /* 占空比 */
}

/* 关闭电机运转 */
void stop_motor(void)
{
	/* 关闭 PWM 输出 */
    timer_channel_output_mode_config(TIMER0,PWM_U_CHANNEL,TIMER_OC_MODE_INACTIVE);
    timer_channel_output_mode_config(TIMER0,PWM_V_CHANNEL,TIMER_OC_MODE_INACTIVE);
    timer_channel_output_mode_config(TIMER0,PWM_W_CHANNEL,TIMER_OC_MODE_INACTIVE);
	/* 上下桥臂全部关断 */
	motor_pwm_set_duty(0,0,0);
}

/* 开启电机运转 */
void start_motor(void)
{
	/* 使能 PWM 输出 */
    timer_channel_output_mode_config(TIMER0,PWM_U_CHANNEL,TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0,PWM_V_CHANNEL,TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0,PWM_W_CHANNEL,TIMER_OC_MODE_PWM0);
}

void bldc_mainfunction(void)
{
    int32_t temp_speed=0;
    if(g_bldc_motor.run_flag == RUN)
    {
        if(g_bldc_motor.dir == CW)                                     /* 正转 */
        {
            g_bldc_motor.step_sta = hallsensor_get_state();     /* 顺序6,2,3,1,5,4 */
        }
        else                                                            /* 反转 */
        {
            g_bldc_motor.step_sta = 7 - hallsensor_get_state(); /* 顺序5,1,3,2,6,4 。使用7减完后可与数组pfunclist_m1对应上顺序 实际霍尔值为：2,6,4,5,1,3*/
        }
        
        if((g_bldc_motor.step_sta <= 6)&&(g_bldc_motor.step_sta >= 1))/* 判断霍尔组合值是否正常 */
        {
            pfunclist_m1[g_bldc_motor.step_sta-1]();                   /* 通过数组成员查找对应的函数指针 */
//            adc_inserted_channel_config(ADC0, IU_INSERTED_CHANNEL, adc_channel[g_bldc_motor.step_sta-1], ADC_SAMPLETIME_1POINT5);
        }
        else                                                            /* 霍尔传感器错误、接触不良、断开等情况 */
        {
            stop_motor();
            g_bldc_motor.run_flag = STOP;
        }
        
        /******************************* 速度计算 *******************************/
        g_bldc_motor.count_j++;                /* 计算速度专用计数值 */
        g_bldc_motor.hall_sta_edge = uemf_edge(g_bldc_motor.step_sta & 0X01);/* 检测单个霍尔信号的变化 */
        if(g_bldc_motor.hall_sta_edge == 0)    /* 统计单个霍尔信号的高电平时间，当只有一对级的时候，旋转一圈为一个完整脉冲。一高一低相加即旋转一圈所花的时间*/
        {
            /*计算速度*/
            if(g_bldc_motor.dir == CW)
                temp_speed = (60*MOTOR_PWM_FREQ_HZ)/g_bldc_motor.count_j/2/MOTOR_NOPOLESPAIRS;
            else
                temp_speed = (60*MOTOR_PWM_FREQ_HZ/g_bldc_motor.count_j/2/MOTOR_NOPOLESPAIRS);
            FirstOrderRC_LPF(g_bldc_motor.speed,temp_speed,0.2379f);   /* 一阶滤波 */
            g_bldc_motor.no_single = 0;
            g_bldc_motor.count_j = 0;
        }
        if(g_bldc_motor.hall_sta_edge == 1)    /* 当采集到下降沿时数据清0 */
        {
            g_bldc_motor.no_single = 0;
            g_bldc_motor.count_j = 0;
        }
        if(g_bldc_motor.hall_sta_edge == 2)    /* 霍尔值一直不变代表未换向 */
        {
            g_bldc_motor.no_single++;          /* 不换相时间累计 超时则判定速度为0 */
            
            if(g_bldc_motor.no_single > 15000)
            {
                stop_motor();
                g_bldc_motor.run_flag = STOP;
                g_bldc_motor.no_single = 0;
                g_bldc_motor.speed = 0;        /* 超时换向 判定为停止 速度为0 */
            }
        }
        /******************************* PID控制 *******************************/
#if(SPEED_LOOP == 1)
        g_bldc_motor.speed_pi.meas = g_bldc_motor.speed;
        g_bldc_motor.speed_pi.ref = g_bldc_motor.target_speed;
        calc_pi(&g_bldc_motor.speed_pi);
        g_bldc_motor.pwm_duty = g_bldc_motor.speed_pi.out;
#endif
    }
}


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/

/**
* @brief 获取霍尔传感器引脚状态
* @param 无
* @retval 霍尔传感器引脚状态
*/
static uint32_t hallsensor_get_state(void)
{
	__IO static uint32_t State ;
	State = 0;
	if(gpio_input_bit_get(HALLA_GPIO_PORT,HALLA_GPIO_PIN) != RESET) /* 霍尔 1 传感器状态获取 */
	{
		State |= 0x01U;
	}
	if(gpio_input_bit_get(HALLB_GPIO_PORT,HALLB_GPIO_PIN) != RESET) /* 霍尔 2 传感器状态获取 */
	{
		State |= 0x02U;
	}
	if(gpio_input_bit_get(HALLC_GPIO_PORT,HALLC_GPIO_PIN) != RESET) /* 霍尔 3 传感器状态获取 */
	{
		State |= 0x04U;
	}
	return State;
}

/**
 * @brief       检测输入信号是否发生变化
 * @param       val :输入信号
 * @note        测量速度使用，获取输入信号状态翻转情况，计算速度
 * @retval      0：计算高电平时间，1：计算低电平时间，2：信号未改变
 */
static uint8_t uemf_edge(uint8_t val)
{
    /* 主要是检测val信号从0 - 1 在从 1 - 0的过程，即高电平所持续的过程 */
    static uint8_t oldval=0;
    if(oldval != val)
    {
        oldval = val;
        if(val == 0) return 0;
        else return 1;
    }
    return 2;
}

/* 上下桥臂的导通情况，共 6 种，也称为 6 步换向 */
void m1_uhvl(void)
{
    motor_pwm_set_duty(g_bldc_motor.pwm_duty,0,0);
    gpio_bit_write(PWMUL_GPIO_PORT, PWMUL_GPIO_PIN, RESET);
    gpio_bit_write(PWMVL_GPIO_PORT, PWMVL_GPIO_PIN, SET);
    gpio_bit_write(PWMWL_GPIO_PORT, PWMWL_GPIO_PIN, RESET);
}

void m1_uhwl(void)
{
    motor_pwm_set_duty(g_bldc_motor.pwm_duty,0,0);
    gpio_bit_write(PWMUL_GPIO_PORT, PWMUL_GPIO_PIN, RESET);
    gpio_bit_write(PWMVL_GPIO_PORT, PWMVL_GPIO_PIN, RESET);
    gpio_bit_write(PWMWL_GPIO_PORT, PWMWL_GPIO_PIN, SET);
}

void m1_vhwl(void)
{
    motor_pwm_set_duty(0,g_bldc_motor.pwm_duty,0);
    gpio_bit_write(PWMUL_GPIO_PORT, PWMUL_GPIO_PIN, RESET);
    gpio_bit_write(PWMVL_GPIO_PORT, PWMVL_GPIO_PIN, RESET);
    gpio_bit_write(PWMWL_GPIO_PORT, PWMWL_GPIO_PIN, SET);
}

void m1_vhul(void)
{
    motor_pwm_set_duty(0,g_bldc_motor.pwm_duty,0);
    gpio_bit_write(PWMUL_GPIO_PORT, PWMUL_GPIO_PIN, SET);
    gpio_bit_write(PWMVL_GPIO_PORT, PWMVL_GPIO_PIN, RESET);
    gpio_bit_write(PWMWL_GPIO_PORT, PWMWL_GPIO_PIN, RESET);
}

void m1_whul(void)
{
    motor_pwm_set_duty(0,0,g_bldc_motor.pwm_duty);
    gpio_bit_write(PWMUL_GPIO_PORT, PWMUL_GPIO_PIN, SET);
    gpio_bit_write(PWMVL_GPIO_PORT, PWMVL_GPIO_PIN, RESET);
    gpio_bit_write(PWMWL_GPIO_PORT, PWMWL_GPIO_PIN, RESET);
}

void m1_whvl(void)
{
    motor_pwm_set_duty(0,0,g_bldc_motor.pwm_duty);
    gpio_bit_write(PWMUL_GPIO_PORT, PWMUL_GPIO_PIN, RESET);
    gpio_bit_write(PWMVL_GPIO_PORT, PWMVL_GPIO_PIN, SET);
    gpio_bit_write(PWMWL_GPIO_PORT, PWMWL_GPIO_PIN, RESET);
}

/***************************************** (END OF FILE) *********************************************/
