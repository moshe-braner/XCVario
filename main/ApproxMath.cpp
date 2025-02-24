/*
 * ApproxMath.cpp
 * by Moshe Braner, 2022, 2024
 */

#include <math.h>
#include "ApproxMath.h"

/* For the purposes used here, trig functions don't need much precision */
/* - on the other hand can save CPU time by using faster approximations */

/* helper function, only valid for positive arguments      */
/* quadratic fit on 0-45 range, results within +-0.25 deg  */
static float atan2_positive(float ns, float ew)
{
  float t;
  if (ew < ns) {
    t = ew / ns;
    //return (45.0*t + 15.86*(t*(1.0-t)));  // ops can be reduced
    return t*((45.0+15.86) - 15.86*t);
  } else {
    t = ns / ew;
    //return (90.0 - 45.0*t - 15.86*(t*(1.0-t)));
    return (90.0 - t*((45.0+15.86) - 15.86*t));
  }
}

/* An approximation to atan2()                         */
/* - returned value is in degrees clockwise-from-North */
/* - arguments are in reverse order from library atan2 */
float atan2_approx(float ns, float ew)
{
  if (ew > 0.0) {
    if (ns > 0.0) return atan2_positive(ns,ew);
    if (ns < 0.0) return 180.0 - atan2_positive(-ns,ew);
    /* if (ns==0) */ return 90.0;
  } else if (ew < 0.0) {
    if (ns > 0.0) return 360.0 - atan2_positive(ns,-ew);
    if (ns < 0.0) return 180.0 + atan2_positive(-ns,-ew);
    /* if (ns==0) */ return 270.0;
  } else {  /* if (ew==0) */
    if (ns >= 0.0) return 0.0;
    return 180.0;
  }
}

// a more accurate formula, but not quite as fast
// https://mazzo.li/posts/vectorized-atan2.html - plus added a constant 0.004965 deg
// max error: +-0.04 deg
// average (signed) error: -0.00000015 deg - not much bias!
// average absolute error: 0.04 deg
// root-mean-square error: 0.03 deg
float atan2_positive_better(float ns, float ew)
{
  float t;
  if (ew < ns) {
    t = ew / ns;
    float s = t*t;
    return (0.004965 + t*(57.02958 - s*(16.540089 - 4.54533*s)));
  } else {
    t = ns / ew;
    float s = t*t;
    return ((90.0-0.004965) - t*(57.02958 - s*(16.540089 - 4.54533*s)));
  }
}
float atan2_better(float ns, float ew)
{
  if (ew > 0.0) {
    if (ns > 0.0) return atan2_positive_better(ns,ew);
    if (ns < 0.0) return 180.0 - atan2_positive_better(-ns,ew);
    /* if (ns==0) */ return 90.0;
  } else if (ew < 0.0) {
    if (ns > 0.0) return 360.0 - atan2_positive_better(ns,-ew);
    if (ns < 0.0) return 180.0 + atan2_positive_better(-ns,-ew);
    /* if (ns==0) */ return 270.0;
  } else {  /* if (ew==0) */
    if (ns >= 0.0) return 0.0;
    return 180.0;
  }
}

// even faster integer version (but cannot avoid one integer division)
// - accurate to within about 1 degree

static int32_t iatan_positive(int32_t ns, int32_t ew)
{
    if (ew > ns)
        return (90 - iatan_positive(ew, ns));
    uint32_t t;
    if (ew < 0x000FFFFF) {
      t = (((uint32_t)ew) << 8);
      t /= (uint32_t)ns;              // 0..256 range since ew <= ns
    } else {
      t = (((uint32_t)ns) >> 8);
      t = (uint32_t)ew / t;
    }
    return (int32_t)((45*t + ((t*(256-t))>>4) + 128)>>8);
}

int32_t iatan2_approx(int32_t ns, int32_t ew)
{
  if (ew > 0) {
    if (ns > 0) return iatan_positive(ns,ew);
    if (ns < 0) return 180 - iatan_positive(-ns,ew);
    /* if (ns==0) */ return 90;
  } else if (ew < 0) {
    if (ns > 0) return 360 - iatan_positive(ns,-ew);
    if (ns < 0) return 180 + iatan_positive(-ns,-ew);
    /* if (ns==0) */ return 270;
  } else {  /* if (ew==0) */
    if (ns >= 0) return 0;
    return 180;
  }
}

/* approximate sin(), argument in degrees, meant for +-360deg range */
/*   https://scholarworks.umt.edu/cgi/viewcontent.cgi?article=1313&context=tme     */
float sin_approx(float degs)
{
  bool neg;
  float prathama, sine;

  neg = (degs < 0.0);
  if (neg)  degs = -degs;
  if (degs > 360.0)  degs = degs - 360.0;
  if (degs > 360.0)  degs = degs - 360.0;
  if (degs > 180.0) {
    prathama = (degs - 180.0) * (360.0 - degs);
    neg = !neg;
  } else {
    prathama = degs * (180.0 - degs);
  }
  //sine = 4.0*prathama / (40500.0-prathama);
  // use the other approximation in same paper to avoid the FP division:
  sine = 3.429355e-9 * (27900.0 + prathama) * prathama;
  if (neg)  return -sine;
  return sine;
}

float cos_approx(float degs)
{
  return sin_approx(degs+90.0);
}

/*
 * Approximate sqrt(x^2+y^2):
 * 
 * Based on: https://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
 * with one added "iteration" (at a cost of a float division).
 * 
 * Maximum error < 0.07%, average error about 0.03%.
 */
float hypot_approx(float x, float y)
{
   x = fabs(x);
   y = fabs(y);
   if (x == 0.0) {
     return y;
   } else if (y == 0.0) {
     return x;
   } else {
     float h;
     if (x < y)  { h=x;  x=y;  y=h; }
     if ( x < 16.0 * y ) {
         h = ( x * (0.983398 - 0.0390625)) + ( y * 0.430664 );
     } else {
         h = ( x * 0.983398 ) + ( y * 0.430664 );
     }
     return (0.5 * ((x*x + y*y) / h + h));
   }
}


// faster integer version (including "iteration"):
//   - faster because integer division instead of float
//   - accuracy about +- 0.05%, similar to float version
uint32_t iapproxHypotenuse1( int32_t x, int32_t y )
{
   uint32_t imin, imax, approx;
   if ( y < 0 ) y = -y;
   if (x == 0)
     return y;
   if ( x < 0 ) x = -x;
   if (y == 0)
     return x;
   if ( x < y ) {
      imin = x;
      imax = y;
   } else {
      imin = y;
      imax = x;
   }
   if (imax < (1<<15)) {
       if ( imax < ( imin << 4 ))
           approx = ( imax * (1007-40) ) + ( imin * 441 );
       else
           approx = ( imax * 1007 ) + ( imin * 441 );
       approx = (( approx + 512 ) >> 10 );   // now in same scale as inputs
       return (((approx + (imin*imin+imax*imax)/approx) + 1) >> 1);
   } else if (imax < (1<<19)) {
       imin   = (( imin   + (1 << 3) ) >> 4 );
       imax   = (( imax   + (1 << 3) ) >> 4 );
       if ( imax < ( imin << 4 ))
           approx = ( imax * (1007-40) ) + ( imin * 441 );
       else
           approx = ( imax * 1007 ) + ( imin * 441 );
       approx = (( approx + 512 ) >> 10 );
       return ((approx + (imin*imin+imax*imax)/approx) << 3);
   } else if (imax < (1<<23)) {
       imin   = (( imin   + (1 << 7) ) >> 8 );
       imax   = (( imax   + (1 << 7) ) >> 8 );
       if ( imax < ( imin << 4 ))
           approx = ( imax * (1007-40) ) + ( imin * 441 );
       else
           approx = ( imax * 1007 ) + ( imin * 441 );
       approx = (( approx + 512 ) >> 10 );
       return ((approx + (imin*imin+imax*imax)/approx) << 7);
   } else if (imax < (1<<27)) {
       imin   = (( imin   + (1 << 11) ) >> 12 );
       imax   = (( imax   + (1 << 11) ) >> 12 );
       if ( imax < ( imin << 4 ))
           approx = ( imax * (1007-40) ) + ( imin * 441 );
       else
           approx = ( imax * 1007 ) + ( imin * 441 );
       approx = (( approx + 512 ) >> 10 );
       return ((approx + (imin*imin+imax*imax)/approx) << 11);
   }
   imin   = (( imin   + (1 << 15) ) >> 16 );
   imax   = (( imax   + (1 << 15) ) >> 16 );
   if ( imax < ( imin << 4 ))
       approx = ( imax * (1007-40) ) + ( imin * 441 );
   else
       approx = ( imax * 1007 ) + ( imin * 441 );
   approx = (( approx + 512 ) >> 10 );
   return ((approx + (imin*imin+imax*imax)/approx) << 15);
}

// even faster but rough integer version (the original code, without "iteration"):
//   - only OK for x, y under about 2^20 in magnitude
//   - accuracy about +- 4%
uint32_t iapproxHypotenuse0( int32_t x, int32_t y )
{
   uint32_t imin, imax, approx;
   if ( y < 0 ) y = -y;
   if (x == 0)
     return y;
   if ( x < 0 ) x = -x;
   if (y == 0)
     return x;
   if ( x < y ) {
      imin = x;
      imax = y;
   } else {
      imin = y;
      imax = x;
   }
   approx = ( imax * 1007 ) + ( imin * 441 );
   if ( imax < ( imin << 4 ))
      approx -= ( imax * 40 );
   // add 512 for proper rounding
   return (( approx + 512 ) >> 10 );
}

// about +-0.01 absolute error:
float log2_approx(float x) {
    int exponent;
    float f = std::frexp(x, &exponent);
    return ((float)exponent + (4 - 1.3433152f * f) * f - 2.6587176f);
}

// https://innovation.ebayinc.com/tech/engineering/fast-approximate-logarithms-part-iii-the-formulas/
// about +-0.001 absolute error:
float log2_approx2(float x) {
    int exponent;
    float f = std::frexp(x, &exponent);
    // range of f: 0.5 to 1.0
    if (f < 0.75) {
        --exponent;
        f = f * 2.0;
    }
    // now range of f: 0.75 to 1.5 -- shift to -0.25 to +0.5:
    f -= 1.0;
    return ((float)exponent + ((0.388531f*f-0.741619f)*f+1.445866f)*f);
}

// helper function, 2^f where f is an integer
static float exp2_int(float f) {
    float x;
    //if (f == 0)
    //    return 1.0;
    int n = (int) f;
    if (n >= 0) {
        if (n < 30)
            return (float)(1<<n);
        x = 2.0;
    } else {  // n < 0
        n = -n;
        x = 0.5;
    }
    float y = 1.0;
    while (n > 1) {
        if (n & 1) {
            y *= x;
            n--;
        }
        x *= x;
        n >>= 1;
    }
    return (x * y);
}

// relative error better than 0.6%
float exp2_approx(float x) {
    float f = floor(x);
    x -= f;
    float p = 1.0+x*(0.6958f+x*0.295f);
    return (p * exp2_int(f));
}

// max relative error about 0.006%
float exp2_approx2(float x) {
    float f = floor(x);
    x -= f;
    float p = 1.0+x*(0.695792359f+x*(0.2251590619f+x*0.0790485792f));
    return (p * exp2_int(f));
}
