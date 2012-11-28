#include "eCompass.h"
#include "MAG3110.h"
#include "trace.h"

// Hard iron estimate
int16_t iVx, iVy, iVz;

char str[32];

void eCompassInit()
{
    uint8_t buf[192*2];
    Mag_Vector_Type magReading;
    
    uint16_t i = 0;
    uint16_t preValues = 100;
    int16_t x[preValues];
    int16_t y[preValues];
    int16_t z[preValues];
    
    // Get values whilst rotating the speck
    while (i < preValues)
    {
        if(MAGRegRead(MAG_DR_STATUS_REG) & MAG_ZYXDR_MASK)
        {
            wait(100);
            TRACE("Getting calibrate values!\n");
            magReading = getMAGReadings();
            
            x[i] = (int16_t) magReading.x;
            y[i] = (int16_t) magReading.y;
            z[i] = (int16_t) magReading.z;
            i++;
        }  
    }
    
    int16_t xMin = x[0];
    int16_t xMax = x[0];
    int16_t yMin = y[0];
    int16_t yMax = y[0];
    int16_t zMin = z[0];
    int16_t zMax = z[0];
    
    // Find max and min of calibration matrix in each axis
    for(i = 1; i < preValues; i++)
    {
        // x
        if (x[i] < xMin)
        {
            xMin = x[i];
        }
        else if (x[i] > xMax)
        {
            xMax = x[i];
        }
        
        // y
        if (y[i] < yMin)
        {
            yMin = y[i];
        }
        else if (y[i] > yMax)
        {
            yMax = y[i];
        }
        
        // z
        if (z[i] < zMin)
        {
            zMin = z[i];
        }
        else if (z[i] > zMax)
        {
            zMax = z[i];
        }

    }
    
    iVx = (int16_t) ((xMax + xMin) / 2);
    iVy = (int16_t) ((yMax + yMin) / 2);
    iVz = (int16_t) ((zMax + zMin) / 2);
    
//    sprintf(str, "iVx = 0x%4.4x %d\niVy = 0x%4.4x %d\niVz = 0x%4.4x %d\n", iVx, iVx, iVy, iVy, iVz, iVz);
//    TRACE(str);
    
}
float ieCompass(int16_t x, int16_t y)
{
    x -= iVx;
    y -= iVy;
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(y ,x);

    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*PI;
   
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/PI; 
    
    headingDegrees = headingDegrees + 110;
    if (headingDegrees > 360) headingDegrees -= 360;
    
    return headingDegrees;
}

float getBearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
        int32_t vecY = lat2 - lat1;
        int32_t vecX = lon2 - lon1;
	
	// Get radians
	float bearing = atan2(vecY, vecX);
	
	// Convert to degrees
	float bearingDeg = bearing *(180/PI);
	
	// First quadrant (xpos, ypos)
	if(bearingDeg >= 0.0 && bearingDeg <= 90.0)
	{
		bearingDeg = 360 - bearingDeg;
	}
	// Second quadrant (xneg, ypos)
	else if(bearingDeg > 90.0 && bearingDeg <= 180.0)
	{
		bearingDeg = 360 - bearingDeg;
	}
	// Third quadrant (xpos, yneg)
	else if(bearingDeg < 0.0 && bearingDeg >= (-90))
	{
		bearingDeg = fabs(bearingDeg) + 90.0;
	}
	// Fourth quadrant (xneg, yneg)
	else if(bearingDeg >= (-180.0) && bearingDeg < (-90))
	{
		bearingDeg = fabs(bearingDeg) + 90.0;
	}

    return bearingDeg;
}