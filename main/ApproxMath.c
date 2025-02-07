/*
 * ApproxMath.cpp
 * by Moshe Braner, 2022, 2024
 */

#include <math.h>
#include <stdio.h>

/* For the purposes used here, trig functions don't need much precision */
/* - on the other hand can save CPU time by using faster approximations */

/* helper function, only valid for positive arguments      */
/* quadratic fit on 0-45 range, results within +-0.25 deg  */
float atan_positive(float ns, float ew)
{
  float t;
  if (ew < ns) {
    t = ew / ns;
    return (45.0*t + 15.5*(t*(1.0-t)));
  } else {
    t = ns / ew;
    return (90.0 - 45.0*t - 15.5*(t*(1.0-t)));
  }
}

/* An approximation to atan2()                         */
/* - returned value is in degrees clockwise-from-North */
/* - arguments are in reverse order from library atan2 */
float atan2_approx(float ns, float ew)
{
  if (ew > 0.0) {
    if (ns > 0.0) return atan_positive(ns,ew);
    if (ns < 0.0) return 180.0 - atan_positive(-ns,ew);
    /* if (ns==0) */ return 90.0;
  } else if (ew < 0.0) {
    if (ns > 0.0) return 360.0 - atan_positive(ns,-ew);
    if (ns < 0.0) return 180.0 + atan_positive(-ns,-ew);
    /* if (ns==0) */ return 270.0;
  } else {  /* if (ew==0) */
    if (ns >= 0.0) return 0.0;
    return 180.0;
  }
}

/* approximate sin(), argument in degrees, meant for +-360deg range */
/*   https://scholarworks.umt.edu/cgi/viewcontent.cgi?article=1313&context=tme     */
float sin_approx(float degs)
{
  int neg;
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

#define PI 3.14159265359
#define D2R (PI / 180.0)
#define R2D (180.0 / PI)

main()
{
    float x, y;

    x = 20;
    printf("x=%.1f sin=%f sin_approx=%f\n", x, sin(D2R*x), sin_approx(x));
    printf("x=%.1f cos=%f cos_approx=%f\n", x, cos(D2R*x), cos_approx(x));

    x = 220;
    printf("x=%.1f sin=%f sin_approx=%f\n", x, sin(D2R*x), sin_approx(x));
    printf("x=%.1f cos=%f cos_approx=%f\n", x, cos(D2R*x), cos_approx(x));

    x = -50;
    printf("x=%.1f sin=%f sin_approx=%f\n", x, sin(D2R*x), sin_approx(x));
    printf("x=%.1f cos=%f cos_approx=%f\n", x, cos(D2R*x), cos_approx(x));

    x = -205;
    printf("x=%.1f sin=%f sin_approx=%f\n", x, sin(D2R*x), sin_approx(x));
    printf("x=%.1f cos=%f cos_approx=%f\n", x, cos(D2R*x), cos_approx(x));

    x = 10;
    y = 20;
    printf("x=%.1f y=%.1f atan(y/x)=%f atan2(y,x)=%f atan2_approx(x,y)=%f\n",
                  x, y, R2D*atan(y/x), R2D*atan2(y,x), atan2_approx(x,y));

    x = -10;
    y = 5;
    printf("x=%.1f y=%.1f atan(y/x)=%f atan2(-y,-x)=%f atan2_approx(-x,-y)=%f\n",
                  x, y, R2D*atan(y/x), R2D*atan2(-y,-x), atan2_approx(-x,-y)-360.0);
    
    x = 30;
    y = -5;
    printf("x=%.1f y=%.1f atan(y/x)=%f atan2(y,x)=%f atan2_approx(x,y)=%f\n",
                  x, y, R2D*atan(y/x), R2D*atan2(y,x), atan2_approx(x,y)-360.0);

    x = 10;
    y = 20;
    printf("x=%.1f y=%.1f hypot(x,y)=%f hypot_approx(x,y)=%f\n",
                  x, y, hypot(x,y), hypot_approx(x,y));

    x = -3;
    y = 20;
    printf("x=%.1f y=%.1f hypot(x,y)=%f hypot_approx(x,y)=%f\n",
                  x, y, hypot(x,y), hypot_approx(x,y));

    x = 50;
    y = -13;
    printf("x=%.1f y=%.1f hypot(x,y)=%f hypot_approx(x,y)=%f\n",
                  x, y, hypot(x,y), hypot_approx(x,y));
}
