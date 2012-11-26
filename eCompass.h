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
#define MINDELTADIV 1
    
// prototypes
    
    void eCompassInit();
    float ieCompass(int16_t x, int16_t y);


#ifdef	__cplusplus
}
#endif

#endif	/* ECOMPASS_H */

