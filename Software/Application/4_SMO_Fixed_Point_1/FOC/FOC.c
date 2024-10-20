#include "FOC.h"

uint16_t SVM_Angle,PWM_Period = 3500;
tSVGenParm 	SVGenParm;
tParkParm 	ParkParm;
tCtrlParm 	CtrlParm;

const int16_t hSin_Cos_hTimePhAble[256] = SIN_COS_TABLE;


void InitPI( tPIParm *pParm)
{
	pParm->qdSum = 0;
	pParm->qOut = 0;
}

void CalcPI( tPIParm *pParm)
{
	int32_t PI_Err,PI_Exc,PI_U,PI_Out;
	PI_Err  = pParm->qInRef - pParm->qInMeas;
	PI_U= pParm->qdSum + pParm->qKp * PI_Err;
	PI_U/=32768;
	
	if( PI_U > pParm->qOutMax ) PI_Out = pParm->qOutMax;
	else if( PI_U < pParm->qOutMin ) PI_Out = pParm->qOutMin;					
	else PI_Out = PI_U;
	
	PI_Exc = PI_U - PI_Out;
	pParm->qdSum = pParm->qdSum + pParm->qKi * PI_Err - pParm->qKc * PI_Exc;
	
	pParm->qOut=(int16_t)PI_Out;	
}
// Calculate qId,qIq from qSin,qCos,qIa,qIb
void ClarkePark(tParkParm* SVM_ClarkePark)
{
	int32_t divSQRT3_tmp;
	int32_t qIdq_tmp_1, qIdq_tmp_2;      
  int16_t qIdq_1, qIdq_2; 
	
  // qIalpha = qIas
  SVM_ClarkePark->qIalpha= SVM_ClarkePark->qIa;
  //qIbeta = (2*qIbs+qIas)/sqrt(3)
  divSQRT3_tmp = SVM_ClarkePark->qIa+ SVM_ClarkePark->qIb+ SVM_ClarkePark->qIb;
	divSQRT3_tmp*=divSQRT_3;
	
  divSQRT3_tmp /=32768;
  SVM_ClarkePark->qIbeta=(int16_t)divSQRT3_tmp; 
  
//-----------------------------Park-------------------------------	
  //No overflow guaranteed
  qIdq_tmp_1 = SVM_ClarkePark->qIbeta * SVM_ClarkePark->qCos;  	
  qIdq_tmp_1 /= 32768;
  
  //No overflow guaranteed
  qIdq_tmp_2 = SVM_ClarkePark->qIalpha *SVM_ClarkePark->qSin;
  qIdq_tmp_2 /= 32768;
 
  qIdq_1 = ((int16_t)(qIdq_tmp_1));
  qIdq_2 = ((int16_t)(qIdq_tmp_2));

  //Iq component in Q1.15 Format 
  SVM_ClarkePark->qIq = ((qIdq_1)-(qIdq_2));	
  
  //No overflow guaranteed
  qIdq_tmp_1 = SVM_ClarkePark->qIbeta * SVM_ClarkePark->qSin;
  qIdq_tmp_1 /= 32768;
  
  //No overflow guaranteed
  qIdq_tmp_2 = SVM_ClarkePark->qIalpha * SVM_ClarkePark->qCos;
  qIdq_tmp_2 /= 32768;
  
  qIdq_1 = ((int16_t)(qIdq_tmp_1));
  qIdq_2 = ((int16_t)(qIdq_tmp_2));				

   //Id component in Q1.15 Format   
  SVM_ClarkePark->qId = ((qIdq_1)+(qIdq_2));
}
// Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
void InvPark(tParkParm* SVM_RevPark)
{ 	
  int32_t qV_tmp1,qV_tmp2;
  int16_t qV_1,qV_2;
   
  //No overflow guaranteed Vb
  qV_tmp1 = SVM_RevPark->qVq * SVM_RevPark->qCos;
  qV_tmp1 /= 32768;
  
  qV_tmp2 = SVM_RevPark->qVd * SVM_RevPark->qSin;
  qV_tmp2 /= 32768;
		
  qV_1 = (int16_t)(qV_tmp1);		
  qV_2 = (int16_t)(qV_tmp2);			

  SVM_RevPark->qVbeta = ((qV_1)+(qV_2));
 
  //Va
  qV_tmp1 = SVM_RevPark->qVq * SVM_RevPark->qSin;
  qV_tmp1 /= 32768;
  
  qV_tmp2 = SVM_RevPark->qVd * SVM_RevPark->qCos;
  qV_tmp2 /= 32768;

  qV_1 = (int16_t)(qV_tmp1);				
  qV_2 = (int16_t)(qV_tmp2);
   				
  SVM_RevPark->qValpha = (-qV_1)+(qV_2);
}
// Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
// Calculate and set PWM duty cycles from Vr1,Vr2,Vr3           
void CalcSVGen (tParkParm* SVM_Volt)
{
   int32_t wX, wY, wZ, wUAlpha, wUBeta,T1W,T2W;
   uint16_t  hTimePhA=0, hTimePhB=0, hTimePhC=0;
   
//wUAlpha=Va*SQRT_3*4*PWM_PERIOD     //Q13 8192

   wUAlpha = SVM_Volt->qValpha * SQRT_3;
	
	 wUAlpha/=2048;

	 wUAlpha=wUAlpha*PWM_Period;
   wUBeta = (SVM_Volt->qVbeta * T);

   wX = wUBeta;
   wY = (-wUBeta + wUAlpha)/2;
   wZ = (-wUBeta - wUAlpha)/2;
   
  // Sector calculation from wX, wY, wZ 
  if( wX >= 0 )
  {       
    // (xx1)
    if( wY >= 0 )
        {
        // (x11)
        // Must be Sector 3 since Sector 7 not allowed
        // Sector 3: (0,1,1)  0-60 degrees
        T2W = wY;
        T1W = wX;
					
        T1W=T1W/4;
        T2W=T2W/4;	
        hTimePhC=(T/8)-((T1W+T2W)/65536);
        hTimePhB=hTimePhC+(T1W/32768);
        hTimePhA=hTimePhB+(T2W/32768);
            
        SVM_Volt->pwma = hTimePhA;
        SVM_Volt->pwmb = hTimePhB;
        SVM_Volt->pwmc = hTimePhC;
        }
    else
        {            
        // (x01)
        if( wZ >= 0 )
            {
            // Sector 5: (1,0,1)  120-180 degrees
            T2W = wX;
            T1W = wZ;
							
            T1W=T1W/4;
            T2W=T2W/4;	
            hTimePhC=(T/8)-((T1W+T2W)/65536);
            hTimePhB=hTimePhC+(T1W/32768);
            hTimePhA=hTimePhB+(T2W/32768);
                
            SVM_Volt->pwma = hTimePhC;
            SVM_Volt->pwmb = hTimePhA;
            SVM_Volt->pwmc = hTimePhB;

            }
        else
            {
            // Sector 1: (0,0,1)  60-120 degrees
            T2W = -wY;
            T1W = -wZ;
							
            T1W=T1W/4;
            T2W=T2W/4;	
            hTimePhC=(T/8)-((T1W+T2W)/65536);
            hTimePhB=hTimePhC+(T1W/32768);
            hTimePhA=hTimePhB+(T2W/32768);

            SVM_Volt->pwma = hTimePhB;
            SVM_Volt->pwmb = hTimePhA;
            SVM_Volt->pwmc = hTimePhC;
            }
        }
  }
  else
  {
    // (xx0)
    if( wY >= 0 )
        {
        // (x10)
        if( wZ >= 0 )
            {
            // Sector 6: (1,1,0)  240-300 degrees
            T2W = wZ;
            T1W = wY;
							
            T1W=T1W/4;
            T2W=T2W/4;	
            hTimePhC=(T/8)-((T1W+T2W)/65536);
            hTimePhB=hTimePhC+(T1W/32768);
            hTimePhA=hTimePhB+(T2W/32768);

            SVM_Volt->pwma = hTimePhB;
            SVM_Volt->pwmb = hTimePhC;
            SVM_Volt->pwmc = hTimePhA;
            }
        else
            {
            // Sector 2: (0,1,0)  300-0 degrees
            T2W = -wZ;
            T1W = -wX;

            T1W=T1W/4;
            T2W=T2W/4;	
            hTimePhC=(T/8)-((T1W+T2W)/65536);
            hTimePhB=hTimePhC+(T1W/32768);
            hTimePhA=hTimePhB+(T2W/32768);							
							
            SVM_Volt->pwma = hTimePhA;
            SVM_Volt->pwmb = hTimePhC;
            SVM_Volt->pwmc = hTimePhB;
            }
        }
    else
        {            
        // (x00)
        // Must be Sector 4 since Sector 0 not allowed
        // Sector 4: (1,0,0)  180-240 degrees
        T2W = -wX;
        T1W = -wY;

        T1W=T1W/4;
        T2W=T2W/4;	
        hTimePhC=(T/8)-((T1W+T2W)/65536);
        hTimePhB=hTimePhC+(T1W/32768);
        hTimePhA=hTimePhB+(T2W/32768);				
					
        SVM_Volt->pwma = hTimePhC;
        SVM_Volt->pwmb = hTimePhB;
        SVM_Volt->pwmc = hTimePhA;

        }
   }
}
// Calculate qSin,qCos from qAngle
void SinCos(tParkParm* Angle_SinCos)
{
  uint16_t hindex;

  /* 10 bit index computation  */  
  hindex = (uint16_t)(Angle_SinCos->qAngle + 32768);  
	hindex /= 64; 
  
  switch (hindex & SIN_MASK) 
  {
  case U0_90:
    Angle_SinCos->qSin = hSin_Cos_hTimePhAble[(uint8_t)(hindex)];
    Angle_SinCos->qCos = hSin_Cos_hTimePhAble[(uint8_t)(0xFF-(uint8_t)(hindex))];
    break;
  
  case U90_180:  
     Angle_SinCos->qSin = hSin_Cos_hTimePhAble[(uint8_t)(0xFF-(uint8_t)(hindex))];
     Angle_SinCos->qCos = -hSin_Cos_hTimePhAble[(uint8_t)(hindex)];
    break;
  
  case U180_270:
     Angle_SinCos->qSin = -hSin_Cos_hTimePhAble[(uint8_t)(hindex)];
     Angle_SinCos->qCos = -hSin_Cos_hTimePhAble[(uint8_t)(0xFF-(uint8_t)(hindex))];
    break;
  
  case U270_360:
     Angle_SinCos->qSin =  -hSin_Cos_hTimePhAble[(uint8_t)(0xFF-(uint8_t)(hindex))];
     Angle_SinCos->qCos =  hSin_Cos_hTimePhAble[(uint8_t)(hindex)]; 
    break;
  default:
    break;
  }
}



