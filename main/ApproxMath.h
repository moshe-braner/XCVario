/*
 * ApproxMath.h
 * by Moshe Braner, 2022
 */

#ifndef APPROXMATH_H
#define APPROXMATH_H

#include "math.h"

float atan2_approx(float, float);
float atan2_better(float, float);
float sin_approx(float);
float cos_approx(float);
float hypot_approx(float, float);

int32_t iatan2_approx( int32_t ns, int32_t ew );
uint32_t iapproxHypotenuse0( int32_t x, int32_t y );
uint32_t iapproxHypotenuse1( int32_t x, int32_t y );

#endif /* APPROXMATH_H */
