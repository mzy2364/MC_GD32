/*
*******************************************************************************************************
*
* �ļ����� : key_fifo.c
* ��    �� : V1.0
* ��    �� : mzy2364
* ˵    �� : ���������ļ�,FIFO���˲��жϷ�ʽ
* 
*******************************************************************************************************
*/


/* ͷ�ļ� -----------------------------------------------------------*/
#include "key_fifo.h"


/* ���� -------------------------------------------------------------*/

static KEY_T key_x[KEY_COUNT];	// �����ṹ������
static KEY_FIFO key_f;			// ����FIFO�ṹ�����

static __IO uint8_t is_init = 0;





/* ��ȡ�����Ƿ��µĺ��� */
static uint8_t IsKey0Down(void) {if(gpio_input_bit_get(KEY0_GPIO_PORT,KEY0_GPIO_PIN)==RESET)return KEY_ISDOWN;else return KEY_NOTDOWN;}




/**
  * @brief ������ʼ��
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
  * @brief ��ȡ�����Ƿ��ʼ��
  * @param None
  * @retval	1-�����Ѿ���ʼ�� 0����û�г�ʼ��
  * @note û�г�ʼ����ʱ����ð���ɨ�費��ȫ
  */
uint8_t key_is_init(void)
{
	return is_init;
}


/**
  * @brief ��ȡ����
  * @param None
  * @retval	���ذ���fifo��һ������
  * @note 
  */
uint8_t key_get(void)
{
	uint8_t ret;
	
	if(key_f.Read == key_f.Write)		
	{
		/* FIFO�� */
		return KEY_NONE;
	}
	else
	{
		/* ��FIFO����һ�������Ҷ�ָ���1 */
		ret = key_f.Buf[key_f.Read++];
		

		if(key_f.Read >= KEY_FIFO_SIZE)
		{
			key_f.Read = 0;
		}
		return ret;
	}
}


/**
  * @brief ��հ���fifo
  * @param None
  * @retval	None
  * @note 
  */
void key_clear(void)
{
	key_f.Read = key_f.Write;
}


/**
  * @brief ������fifo����һ������
  * @param None
  * @retval	None
  * @note 
  */
void key_put(uint8_t keyCode)
{
	/* ��FIFO����һ��������дָ���1 */
	key_f.Buf[key_f.Write++] = keyCode;
	
	if(key_f.Write >= KEY_FIFO_SIZE)
		key_f.Write = 0;
}


/**
  * @brief ɨ�����а���
  * @param None
  * @retval	None
  * @note ���뱻�����Ե���
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
  * @brief ����GPIO��ʼ��
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
  * @brief ����fifo��ؽṹ�������ʼ��
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
	
	/* �жϰ����Ƿ��µĺ��� */
	key_x[0].IsKeyDownFunc = IsKey0Down;
}




/**
  * @brief ���һ������
  * @param None
  * @retval	None
  * @note ������������жϰ����Ƿ���,�������˲�
  */
static void key_detect(uint8_t i)
{
	KEY_T *pKey;
	
	pKey = &key_x[i];
	if(pKey->IsKeyDownFunc() == KEY_ISDOWN)		//����������
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
			/* �˲�������� */
			if(pKey->State == 0)		//�������״̬Ϊδ����
			{	
				pKey->State = 1;		//��ǰ���״̬Ϊ����
				
				key_put((uint8_t)(3*i+1));	//���밴������
			}
			if(pKey->LongTime > 0)		//���֧�ֳ���
			{
				if(pKey->LongCount < pKey->LongTime)
				{
					pKey->LongCount++;
					if(pKey->LongCount >= pKey->LongTime)
					{
						/* ���밴������ */
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
							/* ���������󣬶��ڴ��밴������ */
							pKey->RepeatCount = 0;
							key_put((uint8_t)(3*i+1));
						}
					}
				}
			}
		}
	}
	else		//����δ����
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
			
			if(pKey->State == 1)	//�������ԭ���ǰ��µ�
			{
				
				
				pKey->State = 0;
				
				/* ���밴�������� */
				key_put((uint8_t)(3*i+2));
			}
		}
		
		pKey->LongCount = 0;
		pKey->RepeatCount = 0;
	}
}



/*****************************  (END OF FILE) *********************************/
