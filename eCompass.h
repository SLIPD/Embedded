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
    
#define MINDELTATRIG 1 
#define K1 5701
#define K2 -1645
#define K3 446
#define MINDELTADIV 1
    
// prototypes
    
    int16_t iTrig(int16_t ix, int16_t iy);
    int16_t iHundredAtan2Deg(int16_t iy, int16_t ix);
    int16_t iHundredAtanDeg(int16_t iy, int16_t ix);
    int16_t iDivide(int16_t iy, int16_t ix);
    int16_t ieCompass(int16_t magX, int16_t magY, int16_t magZ, int16_t accelX, int16_t accelY, int16_t accelZ);


#ifdef	__cplusplus
}
#endif

#endif	/* ECOMPASS_H */

