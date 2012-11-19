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
    
/* compassInit()
 * Fills up the calibration matrix vectorArray
 * spin the speck when this function is hot
 */
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

/* findTransformation()
 * Using the calibration matrix, finds the max and min values in each axis (x, y & z)
 * Finds mid point of sphere, and sets offset vector to these values
 */
void findTransformation()
{
    // Max and min vectors of calibration matrix
    vector_t vMax, vMin;
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
}

/* findAverageVectorLength()
 * Using the calibration matrix, finds the average vector components
 */
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

/* transform(vector_t vec)
 * resets vec to origin of sphere, and normalise between 0 and 1
 */
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

float resultantAngle(vector_t vecA, vector_t vecB)
{
    float A = sqrt((vecA.x * vecA.x) + (vecA.y * vecA.y) + (vecA.z * vecA.z));
    float B = sqrt((vecB.x * vecB.x) + (vecB.y * vecB.y) + (vecB.z * vecB.z));
    
    float ab = (vecA.x * vecB.x) + (vecA.y * vecB.y) + (vecA.z * vecB.z);
    
    return acos(ab / (A * B)) * (180 / PI);
}