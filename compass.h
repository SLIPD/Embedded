/* 
 * File:   compass.h
 * Author: GCHAU
 *
 * Created on 19 November 2012, 18:10
 */

#ifndef COMPASS_H
#define	COMPASS_H

#ifdef	__cplusplus
extern "C" {
#endif
 
#define arrSize 500    
    
typedef struct {
  float x;
  float y;
  float z;
} vector_t;

    void compassInit();
    void findTransformation();
    void findAverageVectorLength();
    vector_t transform(vector_t vec);


#ifdef	__cplusplus
}
#endif

#endif	/* COMPASS_H */

