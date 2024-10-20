#ifndef MeasCurr_H
#define MeasCurr_H
#include "system_define.h"

typedef struct 
{
  short   qKa;        // Q15 
  short   Offseta;

  short   qKb;        // Q15 
  short   Offsetb;
} tMeasCurrParm;

//ReadADC.h
typedef struct 
{
  short   qK;         // 1.15 
  short   qADValue;   // 1.15

} tReadADCParm;

void ReadSignedADC0( tReadADCParm* pParm ); // Returns signed value -2*iK -> 2*iK
void InitMeasCompCurr( short Offset_a, short Offset_b );

extern tReadADCParm ReadADCParm;	// Struct used to read ADC values.
extern tMeasCurrParm MeasCurrParm;
#endif

