#ifndef __CORDIC_H
#define __CORDIC_H

#define FASTER
#define RADIANS 1
# define COSCALE 0x11616E8E /* 291597966 = 0.2715717684432241 * 2^30, valid for j>13 */

static long arctantab[] = {
#ifdef DEGREES      /* 16 fractional bits */
# define QUARTER (90L << 16)
# define MAXITER 22 /* the resolution of the arctan table */
    4157273, 2949120, 1740967, 919879, 466945, 234379, 117304, 58666,
    29335, 14668, 7334, 3667, 1833, 917, 458, 229,
    115, 57, 29, 14, 7, 4, 2, 1
#else /* !DEGREES */
# ifdef RADIANS /* 16 fractional bits */
#  define QUARTER ((long)(0.25 * (1L << 16)))	
#  define MAXITER 16    /* the resolution of the arctan table */
		11548,8192,4836,2555,1297,651,326,163,
		81,41,20,10,5,3,1,1,0,0	
#  else /* !RADIANS && !DEGREES */
#  define BRADS 1
#  define QUARTER (1L << 30)
#  define MAXITER 29    /* the resolution of the arctan table */
    756808418, 536870912, 316933406, 167458907, 85004756, 42667331,
    21354465, 10679838, 5340245, 2670163, 1335087, 667544, 333772, 166886,
    83443, 41722, 20861, 10430, 5215, 2608, 1304, 652, 326, 163, 81, 41,
    20, 10, 5, 3, 1
# endif /* !RADIANS && !DEGREES */
#endif /* !DEGREES */

};


/* To rotate a vector through an angle of theta, we calculate:
 *
 *  x' = x cos(theta) - y sin(theta)
 *  y' = x sin(theta) + y cos(theta)
 *
 * The rotate() routine performs multiple rotations of the form:
 *
 *  x[i+1] = cos(theta[i]) { x[i] - y[i] tan(theta[i]) }
 *  y[i+1] = cos(theta[i]) { y[i] + x[i] tan(theta[i]) }
 *
 * with the constraint that tan(theta[i]) = pow(2, -i), which can be
 * implemented by shifting. We always shift by either a positive or
 * negative angle, so the convergence has the ringing property. Since the
 * cosine is always positive for positive and negative angles between -90
 * and 90 degrees, a predictable magnitude scaling occurs at each step,
 * and can be compensated for instead at the end of the iterations by a
 * composite scaling of the product of all the cos(theta[i])'s.
 */


typedef long TFract;    /* f * 2^30 */
TFract FFracMul(TFract a, TFract b);

extern void FxUnitVec(long *cos, long *sin, long theta);
extern void FxRotate(long *argx, long *argy, long theta);
extern long atan2CORDIC(long x, long y);
extern void FxPolarize(long *argx, long *argy);
extern long FFracMul(long a, long b);
extern void PseudoRotate(long *px, long *py, register long theta);
extern void PseudoPolarize(long *argx, long *argy);
#ifndef FASTER
extern int FxPreNorm(long *argx, long *argy);
#endif


#endif /* __CORDIC_H */

