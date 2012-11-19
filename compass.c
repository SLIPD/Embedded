#include "compass.h"
#include "MAG3110.h"
#include "MMA845XQ.h"
#include "trace.h"

#include <string.h>
#include <math.h>
#include <stdint.h>

// Calibration matrix 
vector_t vectorArray [arrSize];

vector_t vectorOffset;
float avgL;
char str [500];
    
void compassInit()
{
    uint8_t buf[6];
    Mag_Vector_Type magReading;
    
    int i = 0;
    while (i < arrSize)
    {
        if(MAGRegRead(DR_STATUS_REG) & ZYXDR_MASK)
        {
            MAGRegReadN(OUT_X_MSB_REG, 6, buf);
            magReading.x = buf[0]<<8 | buf[1];
            magReading.y = buf[2]<<8 | buf[3];
            magReading.z = buf[4]<<8 | buf[5];
            
            vectorArray[i].x = (float) magReading.x;
            vectorArray[i].y = (float) magReading.y;
            vectorArray[i].z = (float) magReading.z;
            i++;
        }
    }
}

void findTransformation()
{
    // Max and min vectors of cal matrix
    vector_t vMax, vMin;

    // Init max and min vectors
    memset(&vMax, 0, sizeof(vector_t));
    memset(&vMin, 0, sizeof(vector_t));
    int i;
    for(i = 0; i < arrSize; i++)
    {
        // x axis
        if(vMin.x > vectorArray[i].x)
        {
            vMin.x = vectorArray[i].x;
        }
        
        if (vMax.x < vectorArray[i].x)
        {
            vMax.x = vectorArray[i].x;
        }
        
        // y axis
        if(vMin.y > vectorArray[i].y)
        {
            vMin.y = vectorArray[i].y;
        }
        
        if (vMax.y < vectorArray[i].y)
        {
            vMax.y = vectorArray[i].y;
        }
        
        // z axis
        if(vMin.z > vectorArray[i].z)
        {
            vMin.z = vectorArray[i].z;
        }
        
        if (vMax.z < vectorArray[i].z)
        {
            vMax.z = vectorArray[i].z;
        }
    }
    
    memset(&vectorOffset, 0, sizeof(vector_t));
    // Calculate vector offsets
    vectorOffset.x = (vMax.x + vMin.x) / 2;
    vectorOffset.y = (vMax.y + vMin.y) / 2;
    vectorOffset.z = (vMax.z + vMin.z) / 2;
    
//    TRACE("vectorOffset ");
//    sprintf(str, " x %d", vectorOffset.x);
//    TRACE(str);
//    sprintf(str, " y %f", vectorOffset.y);
//    TRACE(str);
//    sprintf(str, " z %f\n", vectorOffset.z);
//    TRACE(str);
}

void findAverageVectorLength()
{
    float averageLength = 0;
    int i;
    for(i = 0; i < arrSize; i++)
    {
        averageLength += sqrt((vectorArray[i].x * vectorArray[i].x) + (vectorArray[i].y * vectorArray[i].y) + vectorArray[i].z * vectorArray[i].z);
    }
    averageLength /= arrSize;
    avgL = averageLength;
}

vector_t transform(vector_t vec)
{
    // Resets to origin
    vec.x -= vectorOffset.x;
    vec.y -= vectorOffset.y;
    vec.z -= vectorOffset.z;
    
    // Normalise it between 1 and 0
    vec.x /= avgL;
    vec.y /= avgL;
    vec.z /= avgL;
    
    return vec;
}