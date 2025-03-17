/*
*******************************************************************************************************
*
* 文件名称 : key_fifo.c
* 版    本 : V1.0
* 作    者 : mzy2364
* 说    明 : 按键驱动文件,FIFO加滤波中断方式
* 
*******************************************************************************************************
*/


/* 头文件 -----------------------------------------------------------*/
#include "key_fifo.h"


/* 变量 -------------------------------------------------------------*/

static KEY_T key_x[KEY_COUNT];	// 按键结构体数组
static KEY_FIFO key_f;			// 按键FIFO结构体变量

static __IO uint8_t is_init = 0;





/* 读取按键是否按下的函数 */
static uint8_t IsKey0Down(void) {if(gpio_input_bit_get(KEY0_GPIO_PORT,KEY0_GPIO_PIN)==RESET)return KEY_ISDOWN;else return KEY_NOTDOWN;}




/**
  * @brief 按键初始化
  * @param None
  * @retval	None
  * @note 
  */
void key_fifo_init(void)
{
	key_gpio_config();
	key_var_init();
	is_init = 1;
}

/**
  * @brief 读取按键是否初始化
  * @param None
  * @retval	1-按键已经初始化 0按键没有初始化
  * @note 没有初始化的时候调用按键扫描不安全
  */
uint8_t key_is_init(void)
{
	return is_init;
}


/**
  * @brief 获取按键
  * @param None
  * @retval	返回按键fifo的一个按键
  * @note 
  */
uint8_t key_get(void)
{
	uint8_t ret;
	
	if(key_f.Read == key_f.Write)		
	{
		/* FIFO空 */
		return KEY_NONE;
	}
	else
	{
		/* 从FIFO读出一个数据且读指针加1 */
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
  * @retval	None
  * @note 
  */
void key_clear(void)
{
	key_f.Read = key_f.Write;
}


/**
  * @brief 往按键fifo存入一个按键
  * @param None
  * @retval	None
  * @note 
  */
void key_put(uint8_t keyCode)
{
	/* 往FIFO存入一个数据且写指针加1 */
	key_f.Buf[key_f.Write++] = keyCode;
	
	if(key_f.Write >= KEY_FIFO_SIZE)
		key_f.Write = 0;
}


/**
  * @brief 扫描所有按键
  * @param None
  * @retval	None
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



/**
  * @brief 按键GPIO初始化
  * @param None
  * @retval	None
  * @note 
  */
static void key_gpio_config(void)
{
    rcu_periph_clock_enable(KEY0_GPIO_CLK);

    gpio_init(KEY0_GPIO_PORT,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,KEY0_GPIO_PIN);
}



/**
  * @brief 按键fifo相关结构体变量初始化
  * @param None
  * @retval	None
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
		
	}
	
	/* 判断按键是否按下的函数 */
	key_x[0].IsKeyDownFunc = IsKey0Down;
}




/**
  * @brief 检测一个按键
  * @param None
  * @retval	None
  * @note 这个函数里面判断按键是否按下,并进行滤波
  */
static void key_detect(uint8_t i)
{
	KEY_T *pKey;
	
	pKey = &key_x[i];
	if(pKey->IsKeyDownFunc() == KEY_ISDOWN)		//按键按下了
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
			/* 滤波消抖后的 */
			if(pKey->State == 0)		//如果按键状态为未按下
			{	
				pKey->State = 1;		//标记按键状态为按下
				
				key_put((uint8_t)(3*i+1));	//存入按键按下
			}
			if(pKey->LongTime > 0)		//如果支持长按
			{
				if(pKey->LongCount < pKey->LongTime)
				{
					pKey->LongCount++;
					if(pKey->LongCount >= pKey->LongTime)
					{
						/* 存入按键长按 */
						key_put((uint8_t)(3*i+3));
					}
					
				}
				else
				{
					if(pKey->RepeatSpeed>0)
					{
						pKey->RepeatCount++;
						if(pKey->RepeatCount >= pKey->RepeatSpeed)
						{
							/* 按键长按后，定期存入按键长按 */
							pKey->RepeatCount = 0;
							key_put((uint8_t)(3*i+1));
						}
					}
				}
			}
		}
	}
	else		//按键未按下
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
			
			if(pKey->State == 1)	//如果按键原来是按下的
			{
				
				
				pKey->State = 0;
				
				/* 存入按键弹起来 */
				key_put((uint8_t)(3*i+2));
			}
		}
		
		pKey->LongCount = 0;
		pKey->RepeatCount = 0;
	}
}



/*****************************  (END OF FILE) *********************************/
