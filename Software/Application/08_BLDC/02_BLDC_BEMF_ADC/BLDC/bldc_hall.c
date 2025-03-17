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
/*  �������� */
static void m1_uhvl(void);
static void m1_uhwl(void);
static void m1_vhwl(void);
static void m1_vhul(void);
static void m1_whul(void);
static void m1_whvl(void);

static uint8_t uemf_edge(uint8_t val);
static uint32_t hallsensor_get_state(void);

/* VARIABLES ----------------------------------------------------------------------------------------*/

_bldc_obj g_bldc_motor = {0};   /* ����ṹ���ʼֵ */

/*  ����������ָ������ */
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
 * @brief       BLDC���ƺ���
 * @param       dir :�������, Duty:PWMռ�ձ�
 * @retval      ��
 */
void bldc_ctrl(int32_t dir,float duty)
{
	g_bldc_motor.dir = dir;            /* ���� */
	g_bldc_motor.pwm_duty = duty;      /* ռ�ձ� */
}

/* �رյ����ת */
void stop_motor(void)
{
	/* �ر� PWM ��� */
    timer_channel_output_mode_config(TIMER0,PWM_U_CHANNEL,TIMER_OC_MODE_INACTIVE);
    timer_channel_output_mode_config(TIMER0,PWM_V_CHANNEL,TIMER_OC_MODE_INACTIVE);
    timer_channel_output_mode_config(TIMER0,PWM_W_CHANNEL,TIMER_OC_MODE_INACTIVE);
	/* �����ű�ȫ���ض� */
	motor_pwm_set_duty(0,0,0);
}

/* ���������ת */
void start_motor(void)
{
	/* ʹ�� PWM ��� */
    timer_channel_output_mode_config(TIMER0,PWM_U_CHANNEL,TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0,PWM_V_CHANNEL,TIMER_OC_MODE_PWM0);
    timer_channel_output_mode_config(TIMER0,PWM_W_CHANNEL,TIMER_OC_MODE_PWM0);
}

void bldc_mainfunction(void)
{
    int32_t temp_speed=0;
    if(g_bldc_motor.run_flag == RUN)
    {
        if(g_bldc_motor.dir == CW)                                     /* ��ת */
        {
            g_bldc_motor.step_sta = hallsensor_get_state();     /* ˳��6,2,3,1,5,4 */
        }
        else                                                            /* ��ת */
        {
            g_bldc_motor.step_sta = 7 - hallsensor_get_state(); /* ˳��5,1,3,2,6,4 ��ʹ��7������������pfunclist_m1��Ӧ��˳�� ʵ�ʻ���ֵΪ��2,6,4,5,1,3*/
        }
        
        if((g_bldc_motor.step_sta <= 6)&&(g_bldc_motor.step_sta >= 1))/* �жϻ������ֵ�Ƿ����� */
        {
            pfunclist_m1[g_bldc_motor.step_sta-1]();                   /* ͨ�������Ա���Ҷ�Ӧ�ĺ���ָ�� */
//            adc_inserted_channel_config(ADC0, IU_INSERTED_CHANNEL, adc_channel[g_bldc_motor.step_sta-1], ADC_SAMPLETIME_1POINT5);
        }
        else                                                            /* �������������󡢽Ӵ��������Ͽ������ */
        {
            stop_motor();
            g_bldc_motor.run_flag = STOP;
        }
        
        /******************************* �ٶȼ��� *******************************/
        g_bldc_motor.count_j++;                /* �����ٶ�ר�ü���ֵ */
        g_bldc_motor.hall_sta_edge = uemf_edge(g_bldc_motor.step_sta & 0X01);/* ��ⵥ�������źŵı仯 */
        if(g_bldc_motor.hall_sta_edge == 0)    /* ͳ�Ƶ��������źŵĸߵ�ƽʱ�䣬��ֻ��һ�Լ���ʱ����תһȦΪһ���������塣һ��һ����Ӽ���תһȦ������ʱ��*/
        {
            /*�����ٶ�*/
            if(g_bldc_motor.dir == CW)
                temp_speed = (60*MOTOR_PWM_FREQ_HZ)/g_bldc_motor.count_j/2/MOTOR_NOPOLESPAIRS;
            else
                temp_speed = (60*MOTOR_PWM_FREQ_HZ/g_bldc_motor.count_j/2/MOTOR_NOPOLESPAIRS);
            FirstOrderRC_LPF(g_bldc_motor.speed,temp_speed,0.2379f);   /* һ���˲� */
            g_bldc_motor.no_single = 0;
            g_bldc_motor.count_j = 0;
        }
        if(g_bldc_motor.hall_sta_edge == 1)    /* ���ɼ����½���ʱ������0 */
        {
            g_bldc_motor.no_single = 0;
            g_bldc_motor.count_j = 0;
        }
        if(g_bldc_motor.hall_sta_edge == 2)    /* ����ֵһֱ�������δ���� */
        {
            g_bldc_motor.no_single++;          /* ������ʱ���ۼ� ��ʱ���ж��ٶ�Ϊ0 */
            
            if(g_bldc_motor.no_single > 15000)
            {
                stop_motor();
                g_bldc_motor.run_flag = STOP;
                g_bldc_motor.no_single = 0;
                g_bldc_motor.speed = 0;        /* ��ʱ���� �ж�Ϊֹͣ �ٶ�Ϊ0 */
            }
        }
        /******************************* PID���� *******************************/
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
* @brief ��ȡ��������������״̬
* @param ��
* @retval ��������������״̬
*/
static uint32_t hallsensor_get_state(void)
{
	__IO static uint32_t State ;
	State = 0;
	if(gpio_input_bit_get(HALLA_GPIO_PORT,HALLA_GPIO_PIN) != RESET) /* ���� 1 ������״̬��ȡ */
	{
		State |= 0x01U;
	}
	if(gpio_input_bit_get(HALLB_GPIO_PORT,HALLB_GPIO_PIN) != RESET) /* ���� 2 ������״̬��ȡ */
	{
		State |= 0x02U;
	}
	if(gpio_input_bit_get(HALLC_GPIO_PORT,HALLC_GPIO_PIN) != RESET) /* ���� 3 ������״̬��ȡ */
	{
		State |= 0x04U;
	}
	return State;
}

/**
 * @brief       ��������ź��Ƿ����仯
 * @param       val :�����ź�
 * @note        �����ٶ�ʹ�ã���ȡ�����ź�״̬��ת����������ٶ�
 * @retval      0������ߵ�ƽʱ�䣬1������͵�ƽʱ�䣬2���ź�δ�ı�
 */
static uint8_t uemf_edge(uint8_t val)
{
    /* ��Ҫ�Ǽ��val�źŴ�0 - 1 �ڴ� 1 - 0�Ĺ��̣����ߵ�ƽ�������Ĺ��� */
    static uint8_t oldval=0;
    if(oldval != val)
    {
        oldval = val;
        if(val == 0) return 0;
        else return 1;
    }
    return 2;
}

/* �����ű۵ĵ�ͨ������� 6 �֣�Ҳ��Ϊ 6 ������ */
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
