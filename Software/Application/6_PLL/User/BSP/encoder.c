/*
*******************************************************************************************************
*
* File Name : encoder.c
* Version   : V1.0
* Author    : mzy2364
* brief     : encoder file
* 
*******************************************************************************************************
*/


/* INCLUDE FILES ------------------------------------------------------------------------------------*/
#include "encoder.h"

/* DEFINES ------------------------------------------------------------------------------------------*/


/* VARIABLES ----------------------------------------------------------------------------------------*/
static encoder_code_t encoder_code = ENCODER_NONE;
static KEY_T key_x[KEY_COUNT];  // 按键结构体数组
static KEY_FIFO key_f;          // 按键FIFO结构体变量

/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
static void encoder_gpio_init(void);
static void encoder_timer_init(void);

static void key_put(uint8_t keyCode);
static void key_var_init(void);
static void key_detect(uint8_t i);
/* 读取按键是否按下的函数 */
static uint8_t IsKey0Down(void) {if(gpio_input_bit_get(KEY_GPIO_PORT,KEY_GPIO_PIN)==0)return KEY_ISDOWN;else return KEY_NOTDOWN;}


/* FUNCTION -----------------------------------------------------------------------------------------*/
/**
  * @brief encoder timer init
  * @param None
  * @retval None
  */
void encoder_init(void)
{
    nvic_irq_enable(TIMER3_IRQn, 1, 1);
    encoder_gpio_init();
    encoder_timer_init();
    key_var_init();
}

encoder_code_t encoder_get(void)
{
    encoder_code_t code = encoder_code;
    encoder_code = ENCODER_NONE;
    return code;
}

/**
  * @brief 获取按键
  * @param None
  * @retval 返回按键fifo的一个按键
  * @note 
  */
uint8_t key_get(void)
{
    uint8_t ret;
    
    if(key_f.Read == key_f.Write)       
    {
        //FIFO空
        return KEY_NONE;
    }
    else
    {
        //从FIFO读出一个数据且读指针加1
        ret = key_f.Buf[key_f.Read++];
        

        if(key_f.Read >= KEY_FIFO_SIZE)
        {
            key_f.Read = 0;
        }
        return ret;
    }
}

/**
  * @brief 清空按键fifo
  * @param None
  * @retval None
  * @note 
  */
void key_clear(void)
{
    key_f.Read = key_f.Write;
}


/**
  * @brief 扫描所有按键
  * @param None
  * @retval None
  * @note 必须被周期性调用
  */
void key_scan(void)
{
    uint8_t i=0;
    for(i=0;i<KEY_COUNT;i++)
    {
        key_detect(i);
    }
}

void TIMER3_IRQHandler(void)
{
    int16_t cnt = 0;
    if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_TRG))
    {
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_TRG);
        cnt = timer_counter_read(TIMER3);
        timer_counter_value_config(TIMER3,0);
        if(cnt > 0)
        {
            encoder_code = ENCODER_INC;
        }
        else
        {
            encoder_code = ENCODER_DEC;
        }
    }
}


/* LOCAL FUNCTION -----------------------------------------------------------------------------------*/
/**
  * @brief 
  * @param None
  * @retval None
  */
static void encoder_gpio_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(ENCODERA_GPIO_CLK);
    rcu_periph_clock_enable(ENCODERB_GPIO_CLK);
    rcu_periph_clock_enable(KEY_GPIO_CLK);

    gpio_init(ENCODERA_GPIO_PORT,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,ENCODERA_GPIO_PIN);
    gpio_init(ENCODERB_GPIO_PORT,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,ENCODERB_GPIO_PIN);
    gpio_init(KEY_GPIO_PORT,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,KEY_GPIO_PIN);
}


static void encoder_timer_init(void)
{
    /* TIMER3 configuration: input capture mode */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);

    /* TIMER3 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3,&timer_initpara);

    /* TIMER3  configuration */
    /* TIMER3 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x10;
    timer_input_capture_config(TIMER3,TIMER_CH_0,&timer_icinitpara);
    timer_input_capture_config(TIMER3,TIMER_CH_1,&timer_icinitpara);

    timer_input_trigger_source_select(TIMER3,TIMER_SMCFG_TRGSEL_CI0FE0);
    timer_quadrature_decoder_mode_config(TIMER3,TIMER_ENCODER_MODE2,TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_RISING);
    
    timer_interrupt_enable(TIMER3,TIMER_INT_TRG);

    /* TIMER2 counter enable */
    timer_enable(TIMER3);
}

/**
  * @brief 往按键fifo存入一个按键
  * @param None
  * @retval None
  * @note 
  */
static void key_put(uint8_t keyCode)
{
    //往FIFO存入一个数据且写指针加1
    key_f.Buf[key_f.Write++] = keyCode;
    
    if(key_f.Write >= KEY_FIFO_SIZE)
        key_f.Write = 0;
}

/**
  * @brief 按键fifo相关结构体变量初始化
  * @param None
  * @retval None
  * @note 
  */
static void key_var_init(void)
{
    uint8_t i=0;
    
    key_f.Read = 0;
    key_f.Write = 0;
    
    for(i=0;i<KEY_COUNT;i++)
    {
        key_x[i].LongTime = KEY_LONG_TIME;
        key_x[i].Count = KEY_FILTER_TIME / 2;
        key_x[i].State = 0;
        key_x[i].RepeatSpeed = 0;
        key_x[i].RepeatCount = 0;
        key_x[i].LongPressFlag = 0;
    }
    
    //判断按键是否按下的函数
    key_x[0].IsKeyDownFunc = IsKey0Down;
}




/**
  * @brief 检测一个按键
  * @param None
  * @retval None
  * @note 这个函数里面判断按键是否按下,并进行滤波
  */
static void key_detect(uint8_t i)
{
    KEY_T *pKey;
    
    pKey = &key_x[i];
    if(pKey->IsKeyDownFunc() == KEY_ISDOWN)     //按键按下了
    {
        if(pKey->Count < KEY_FILTER_TIME)
        {
            pKey->Count = KEY_FILTER_TIME;
        }
        else if(pKey->Count < 2 * KEY_FILTER_TIME)
        {
            pKey->Count++;
        }
        else
        {
            //滤波消抖后的
            if(pKey->State == 0)        //如果按键状态为未按下
            {   
                pKey->State = 1;        //标记按键状态为按下
                
                key_put((uint8_t)(3*i+1));  //存入按键按下
            }
            if(pKey->LongTime > 0)      //如果支持长按
            {
                if(pKey->LongCount < pKey->LongTime)
                {
                    pKey->LongCount++;
                    if(pKey->LongCount >= pKey->LongTime)
                    {
                        //存入按键长按
                        key_put((uint8_t)(3*i+3));
                        pKey->LongPressFlag = 1;
                    }
                    
                }
                else
                {
                    if(pKey->RepeatSpeed>0)
                    {
                        pKey->RepeatCount++;
                        if(pKey->RepeatCount >= pKey->RepeatSpeed)
                        {
                            //按键长按后，定期存入按键长按
                            pKey->RepeatCount = 0;
                            key_put((uint8_t)(3*i+1));
                        }
                    }
                }
            }
        }
    }
    else        //按键未按下
    {   
        if(pKey->Count > KEY_FILTER_TIME)
        {
            pKey->Count = KEY_FILTER_TIME;
        }
        else if(pKey->Count > 0)
        {
            pKey->Count--;
        }
        else
        {
            
            if(pKey->State == 1)    //如果按键原来是按下的
            {
                pKey->State = 0;
                if(pKey->LongPressFlag)
                {
                    //存入按键长按后释放

                    pKey->LongPressFlag = 0;
                }
                else
                {
                    //存入按键弹起来
                    key_put((uint8_t)(3*i+2));
                }
            }
        }
        
        pKey->LongCount = 0;
        pKey->RepeatCount = 0;
    }
}

/***************************************** (END OF FILE) *********************************************/
