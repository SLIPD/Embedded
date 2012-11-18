/* 
 * File:   eCompass.h
 * Author: GCHAU
 *
 * Created on 18 November 2012, 12:56
 */

#ifndef ECOMPASS_H
#define	ECOMPASS_H

#include <stdint.h>
#include <math.h>
#include <stddef.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
// consts
    
const uint16_t MINDELTATRIG = 1; /* final step size for iTrig */
const int16_t K1 = 5701;
const int16_t K2 = -1645;
const int16_t K3 = 446;
const uint16_t MINDELTADIV = 1;
    
// prototypes
    
    int16_t iTrig(int16_t ix, int16_t iy);
    int16_t iHundredAtan2Deg(int16_t iy, int16_t ix);
    int16_t iHundredAtanDeg(int16_t iy, int16_t ix);
    int16_t iDivide(int16_t iy, int16_t ix);


#ifdef	__cplusplus
}
#endif

#endif	/* ECOMPASS_H */

