#include "cordic.h"
					
void PseudoRotate(long *px, long *py, register long theta)
{
    register int i;
    register long x, y, xtemp;
    register long *arctanptr;

    x = *px;
    y = *py;

    /* Get angle between -90 and 90 degrees */
    while (theta < -QUARTER) {
        x = -x;
        y = -y;
        theta += 2 * QUARTER;
    }
    while (theta > QUARTER) {
        x = -x;
        y = -y;
        theta -= 2 * QUARTER;
    }

    /* Initial pseudorotation, with left shift */
    arctanptr = arctantab;
    if (theta < 0) {
        xtemp = x + (y << 1);
        y     = y - (x << 1);
        x     = xtemp;
        theta += *arctanptr++;
    }
    else {
        xtemp = x - (y << 1);
        y     = y + (x << 1);
        x     = xtemp;
        theta -= *arctanptr++;
    }

    /* Subsequent pseudorotations, with right shifts */
    for (i = 0; i <= MAXITER; i++) {
        if (theta < 0) {
            xtemp = x + (y >> i);
            y     = y - (x >> i);
            x     = xtemp;
            theta += *arctanptr++;
        }
        else {
            xtemp = x - (y >> i);
            y     = y + (x >> i);
            x     = xtemp;
            theta -= *arctanptr++;
        }
    }

    *px = x;
    *py = y;
}


void PseudoPolarize(long *argx, long *argy)
{
    register long theta;
    register long yi, i;
    register long x, y;
    register long *arctanptr;

    x = *argx;
    y = *argy;

    /* Get the vector into the right half plane */
    theta = 0;
    if (x < 0) {
        x = -x;
        y = -y;
        theta = 2 * QUARTER;
    }

    if (y > 0)
        theta = - theta;
    
    arctanptr = arctantab;
    if (y < 0) {    /* Rotate positive */
        yi = y + (x << 1);
        x  = x - (y << 1);
        y  = yi;
        theta -= *arctanptr++;  /* Subtract angle */
    }
    else {      /* Rotate negative */
        yi = y - (x << 1);
        x  = x + (y << 1);
        y  = yi;
        theta += *arctanptr++;  /* Add angle */
    }

    for (i = 0; i <= MAXITER; i++) {
        if (y < 0) {    /* Rotate positive */
            yi = y + (x >> i);
            x  = x - (y >> i);
            y  = yi;
            theta -= *arctanptr++;
        }
        else {      /* Rotate negative */
            yi = y - (x >> i);
            x  = x + (y >> i);
            y  = yi;
            theta += *arctanptr++;
        }
    }

    *argx = x;
    *argy = theta;
}


#ifndef FASTER
/* FxPreNorm() block normalizes the arguments to a magnitude suitable for
 * CORDIC pseudorotations.  The returned value is the block exponent.
 */
int FxPreNorm(long *argx, long *argy)
{
    register long x, y;
    int signx, signy;
    register int shiftexp;

    shiftexp = 0;       /* Block normalization exponent */
    signx = signy = 1;

    if ((x = *argx) < 0) {
        x = -x;
        signx = -signx;
    }
    if ((y = *argy) < 0) {
        y = -y;
        signy = -signy;
    }
    /* Prenormalize vector for maximum precision */
    if (x < y) {    /* |y| > |x| */
        while (y < (1 << 27)) {
            x <<= 1;
            y <<= 1;
            shiftexp--;
        }
        while (y > (1 << 28)) {
            x >>= 1;
            y >>= 1;
            shiftexp++;
        }
    }
    else {      /* |x| > |y| */
        while (x < (1 << 27)) {
            x <<= 1;
            y <<= 1;
            shiftexp--;
        }
        while (x > (1 << 28)) {
            x >>= 1;
            y >>= 1;
            shiftexp++;
        }
    }

    *argx = (signx < 0) ? -x : x;
    *argy = (signy < 0) ? -y : y;
    return(shiftexp);
}
#endif // FASTER


/* Return a unit vector corresponding to theta.
 * sin and cos are fixed-point fractions.
 */
void FxUnitVec(long *cos, long *sin, long theta)
{
    *cos = COSCALE;
    *sin = 0;
    PseudoRotate(cos, sin, theta);
}

/* Fxrotate() rotates the vector (argx, argy) by the angle theta. */
void FxRotate(long *argx, long *argy, long theta)
{
#ifndef FASTER
    int shiftexp;
#endif // FASTER

    if (((*argx == 0) && (*argy == 0)) || (theta == 0))
        return;

#ifndef FASTER
    shiftexp = FxPreNorm(argx, argy);  /* Prenormalize vector */
#endif // FASTER
    PseudoRotate(argx, argy, theta);   /* Perform CORDIC pseudorotation */
    *argx = FFracMul(*argx, COSCALE);   /* Compensate for CORDIC enlargement */
    *argy = FFracMul(*argy, COSCALE);
#ifndef FASTER
    if (shiftexp < 0) {     /* Denormalize vector */
        *argx >>= -shiftexp;
        *argy >>= -shiftexp;
    }
    else {
        *argx <<= shiftexp;
        *argy <<= shiftexp;
    }
#endif // FASTER
}

long atan2CORDIC(long x, long y)
{
	if ((x == 0) && (y == 0))return(0);

#ifndef FASTER
	FxPreNorm(&y, &x);  /* Prenormalize vector for maximum precision */
#endif // FASTER
	PseudoPolarize(&y, &x); /* Convert to polar coordinates */
				
	return(x);
}


void FxPolarize(long *argx, long *argy)
{
#ifndef FASTER
    int shiftexp;
#endif // FASTER

    if ((*argx == 0) && (*argy == 0)) {
        return;
    }

#ifndef FASTER
    /* Prenormalize vector for maximum precision */
    shiftexp = FxPreNorm(argx, argy);
#endif // FASTER

    /* Perform CORDIC conversion to polar coordinates */
    PseudoPolarize(argx, argy);

    /* Scale radius to undo pseudorotation enlargement factor */
    *argx = FFracMul(*argx, COSCALE);

#ifndef FASTER
    /* Denormalize radius */
    *argx = (shiftexp < 0) ? (*argx >> -shiftexp) : (*argx << shiftexp);
#endif // FASTER
}

long FFracMul(long a, long b)        /* Just for testing */
{
    /* This routine should be written in assembler to calculate the
     * high part of the product, i.e. the product when the operands
     * are considered fractions.
     */
    return((a >> 15) * (b >> 15));
}
