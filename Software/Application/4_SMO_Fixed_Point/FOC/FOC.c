#include "foc.h"

#define step(shift) \
	if((0x40000000l >> shift) + root <= value)          \
	{                                                   \
	    value -= (0x40000000l >> shift) + root;         \
	    root = (root >> 1) | (0x40000000l >> shift);    \
	}                                                   \
	else                                                \
	{                                                   \
	    root = root >> 1;                               \
	}

static const int16_t hSin_Cos_table[256] = SIN_COS_TABLE;


/**
  * @brief 
  * @param None
  * @retval None
  * @note for one shunt,Prevent duty cycles from being equal,tb must be increased by 1
  */
void svpwm_gen(pmsm_foc_t *foc)
{
    int32_t vr1 = 0, vr2 = 0, vr3 = 0, valpha = 0, vbeta = 0, t1w = 0, t2w = 0;
    uint16_t ta = 0, tb = 0, tc = 0;

    valpha = foc->valpha * SQRT3_BY2;
    valpha = RIGHSHIFT15(valpha);

    vr1 = foc->vbeta;
    vr2 = -foc->vbeta/2 + valpha;
    vr3 = -foc->vbeta/2 - valpha;

    if( vr1 >= 0 )
    {
        // (xx1)
        if( vr2 >= 0 )
        {
            t1w = foc->pwm_period * vr2;
            t2w = foc->pwm_period * vr1;

            tc = (foc->pwm_period - ((t1w + t2w)/32768))/2;
            tb = tc + (t2w / 32768);
            ta = tb + (t1w / 32768);

            foc->pwma = ta;
            foc->pwmb = tb;
            foc->pwmc = tc;

            foc->sector = 1;
        }
        else
        {
            // (x01)
            if( vr3 >= 0 )
            {
                t1w = foc->pwm_period * vr1;
                t2w = foc->pwm_period * vr3;

                tc = (foc->pwm_period - ((t1w + t2w)/32768))/2;
                tb = tc + (t2w / 32768);
                ta = tb + (t1w / 32768);

                foc->pwma = tc;
                foc->pwmb = ta;
                foc->pwmc = tb;

                foc->sector = 3;
            }
            else
            {
                t1w = foc->pwm_period * -vr2;
                t2w = foc->pwm_period * -vr3;

                tc = (foc->pwm_period - ((t1w + t2w)/32768))/2;
                tb = tc + (t2w / 32768);
                ta = tb + (t1w / 32768);

                foc->pwma = tb;
                foc->pwmb = ta;
                foc->pwmc = tc;

                foc->sector = 2;
            }
        }
    }
    else
    {
        // (xx0)
        if( vr2 >= 0 )
        {
            // (x10)
            if( vr3 >= 0 )
            {
                t1w = foc->pwm_period * vr3;
                t2w = foc->pwm_period * vr2;

                tc = (foc->pwm_period - ((t1w + t2w)/32768))/2;
                tb = tc + (t2w / 32768);
                ta = tb + (t1w / 32768);

                foc->pwma = tb;
                foc->pwmb = tc;
                foc->pwmc = ta;

                foc->sector = 5;
            }
            else
            {
                t1w = foc->pwm_period * -vr3;
                t2w = foc->pwm_period * -vr1;

                tc = (foc->pwm_period - ((t1w + t2w)/32768))/2;
                tb = tc + (t2w / 32768);
                ta = tb + (t1w / 32768);

                foc->pwma = ta;
                foc->pwmb = tc;
                foc->pwmc = tb;

                foc->sector = 6;
            }
        }
        else
        {
            t1w = foc->pwm_period * -vr1;
            t2w = foc->pwm_period * -vr2;

            tc = (foc->pwm_period - ((t1w + t2w)/32768))/2;
            tb = tc + (t2w / 32768);
            ta = tb + (t1w / 32768);

            foc->pwma = tc;
            foc->pwmb = tb;
            foc->pwmc = ta;

            foc->sector = 4;
        }
    }
}

/**
  * @brief clark
  * @param None
  * @retval None
  * @note ialpha = ia
  *       ibeta = (ia+2ib)/sqrt(3)
  */
void clark(pmsm_foc_t *foc)
{
	int16_t ibeta = 0;
	int32_t vel_temp = 0;
	
	vel_temp = (foc->ia + foc->ib * 2) * INV_SQRT3;
	ibeta = RIGHSHIFT15(vel_temp);
	
	foc->ialpha = foc->ia;
	foc->ibeta = ibeta;
}

/**
  * @brief park
  * @param None
  * @retval None
  * @note Id = ialpha*cos(theta) + ibeta*sin(theta)
  *       Iq = -ialpha*sin(theta) + ibeta*cos(theta)
  */
void park(pmsm_foc_t *foc)
{
	int32_t vel_temp = 0;

	vel_temp = (foc->ialpha * foc->cos) + (foc->ibeta * foc->sin);
	foc->id = RIGHSHIFT15(vel_temp);
	vel_temp = (foc->ibeta * foc->cos) - (foc->ialpha * foc->sin);
	foc->iq = RIGHSHIFT15(vel_temp);
}

/**
  * @brief inv_park
  * @param None
  * @retval None
  * @note valpha = vd*cos(theta) - vq*sin(theta)
  *       vbeta  = vd*sin(theta) + vq*cos(theta)
  */
void inv_park(pmsm_foc_t *foc)
{
	int32_t vel_temp = 0;
	
	vel_temp = (foc->vd * foc->cos) - (foc->vq * foc->sin);
	foc->valpha = RIGHSHIFT15(vel_temp);
	vel_temp = (foc->vd * foc->sin) + (foc->vq * foc->cos);
	foc->vbeta = RIGHSHIFT15(vel_temp);
}

/**
  * @brief sin cos calculate
  * @param None
  * @retval None
  * @note 
  */
void sin_cos(pmsm_foc_t *foc)
{
	uint16_t hindex = 0;

	/* 10 bit index computation  */  
	hindex = (uint16_t)(foc->angle + 32768);  
	hindex /= 64; 

	switch (hindex & SIN_MASK) 
	{
		case U0_90:
			foc->sin = hSin_Cos_table[(uint8_t)(hindex)];
			foc->cos = hSin_Cos_table[(uint8_t)(0xFF-(uint8_t)(hindex))];
			break;
		case U90_180:  
			foc->sin = hSin_Cos_table[(uint8_t)(0xFF-(uint8_t)(hindex))];
			foc->cos = -hSin_Cos_table[(uint8_t)(hindex)];
			break;
		case U180_270:
			foc->sin = -hSin_Cos_table[(uint8_t)(hindex)];
			foc->cos = -hSin_Cos_table[(uint8_t)(0xFF-(uint8_t)(hindex))];
			break;
		case U270_360:
			foc->sin =  -hSin_Cos_table[(uint8_t)(0xFF-(uint8_t)(hindex))];
			foc->cos =  hSin_Cos_table[(uint8_t)(hindex)]; 
			break;
		default:
			break;
	}
}

/**
  * @brief init pi 
  * @param None
  * @retval None
  * @note 
  */
void init_pi(pid_control_t *p_parm)
{
	p_parm->sum = 0;
	p_parm->out = 0;
}

/**
  * @brief calculate pi 
  * @param None
  * @retval None
  * @note 
  */
void calc_pi(pid_control_t *p_parm)
{
	int32_t PI_Err,PI_Exc,PI_U,PI_Out;
	PI_Err  = p_parm->ref - p_parm->meas;
	PI_U= p_parm->sum + p_parm->kp * PI_Err;
	PI_U/=32768;
	
	if( PI_U > p_parm->out_max ) PI_Out = p_parm->out_max;
	else if( PI_U < p_parm->out_min ) PI_Out = p_parm->out_min;					
	else PI_Out = PI_U;
	
	PI_Exc = PI_U - PI_Out;
	p_parm->sum = p_parm->sum + p_parm->ki * PI_Err - p_parm->kc * PI_Exc;
	
	p_parm->out=(int16_t)PI_Out;	
}

/**
  * @brief Q15 square root function
  * @param data:input data
  * @retval square root of input value
  * @note 
  */
int32_t sqrt_q15(int32_t value)
{
	long root = 0;
	
    step( 0);
    step( 2);
    step( 4);
    step( 6);
    step( 8);
    step(10);
    step(12);
    step(14);
    step(16);
    step(18);
    step(20);
    step(22);
    step(24);
    step(26);
    step(28);
    step(30);

    // round to the nearest integer, cuts max error in half
	if(root < value)++root;
	return root;
}

