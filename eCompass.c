#include "eCompass.h"
#include "MAG3110.h"
#include "trace.h"

// Source: AN4248


// Roll, pitch and yaw angles computed by 1ecompass
int16_t iPhi, iThe, iPsi;
// Magnetic field readings corrected for hard iron effects and PCB orientation
int16_t iBfx, iBfy, iBfz;
// Hard iron estimate
int16_t iVx, iVy, iVz;

char trace_str [300];

const int16_t K1 = 5701;
const int16_t K2 = -1645;
const int16_t K3  = 446;

void eCompassInit()
{
    uint8_t buf[192*2];
    Mag_Vector_Type magReading;
    
    uint16_t i = 0;
    uint16_t preValues = 500;
    int16_t x[preValues];
    int16_t y[preValues];
    int16_t z[preValues];
    
    // Get values whilst rotating the speck
    while (i < preValues)
    {
        if(MAGRegRead(MAG_DR_STATUS_REG) & MAG_ZYXDR_MASK)
        {
            TRACE("Getting values!\n");
            magReading.x = MAGReadX_16();
            magReading.y = MAGReadY_16();
            magReading.z = MAGReadZ_16();
            char str[32];
            sprintf(str, "magReading.x 0x%4.4x %d\n", magReading.x, magReading.x);
            TRACE(str);
            sprintf(str, "magReading.y 0x%4.4x %d\n", magReading.y, magReading.y);
            TRACE(str);
            sprintf(str, "magReading.z 0x%4.4x %d\n", magReading.z, magReading.z);
            TRACE(str);
            
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
    
    sprintf(trace_str, "iVx = 0x%4.4x %d\niVy = 0x%4.4x %d\niVz = 0x%4.4x %d\n", iVx, iVx, iVy, iVy, iVz, iVz);
    TRACE(trace_str);
    
    
//    writeMagOffset((uint16_t) iVx, (uint16_t) iVy, (uint16_t) iVz);
}

float lolCompass(int16_t x, int16_t y)
{
    float heading;
    
    // Hard iron off setting here if done
    x -= iVx;
    y -= iVy;
    char str[32];
    sprintf(str, "x %d, y %d\n",x, y);
    TRACE(str);
    
    if(y > 0) 
    {
        heading = 90 - atan((float)x/(float)y)*(180/PI);
    }
    else if (y < 0)
    {
        heading = 270 - atan((float)x/(float)y)*(180/PI);
    }
    else if(y = 0 & x < 0)
    {
        heading = 180;
    }
    else if(y = 0, x > 0)
    {
        heading = 0;
    }
    else
    {
        TRACE("BROKEN\n");
    }
    return heading;
}

float iCompass(int16_t x, int16_t y)
{
    x -= iVx;
    y -= iVy;
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(y ,x);

    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*PI;
    char str[32];
    sprintf(str, "heading %f\n",heading);
    TRACE(str);
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/PI; 
    headingDegrees = headingDegrees + 270;
    if (headingDegrees > 360) headingDegrees -= 360;
    sprintf(str, "headingDegrees %f\n",headingDegrees);
    TRACE(str);
    return headingDegrees;
}

/* iTrig
 * ix, iy: int16_t representing sensor reading in range of -32768 to 32767
 * function returns int16_t as signed fraction
 * (+32767 = 0.99997, -32768 = -1.0000
 * algorithm solves for ir*ir*(ix*ix+iy*iy) = ix*ixakin
 */
int16_t iTrig(int16_t ix, int16_t iy)
{
    uint32_t    itmp;          // scratch
    uint32_t    ixsq;           // ix * ix
    int16_t     isignx;         // storage for sign of x. x >= 0
    uint32_t    ihypsq;         // (ix * ix) + (iy * iy)
    int16_t     ir;             // result = ix / sqrt(ix * ix + iy * iy
    int16_t     idelta;         // delta on candidate result 
    
    if ((ix == 0) && (iy == 0))
    {
        ix = 1;
        iy = 1;
    }
    
    // Check for -32768 which isn't handled correctly
    if (ix == -32768)
    {
        ix = -32767;
    }
    
    if (iy == -32768)
    {
        iy = -32767;
    }
    
    // Store sign for later use, algorithm assumes x >= 0
    isignx = 1;
    if (ix < 0)
    {
        ix = (int16_t) -ix;
        isignx = -1;
    }
    
    // set iy to be positive
    iy = (int16_t)abs(iy);
    
    // reduce quantization effects, boost ix and iy but keep below int16_t
    while ((ix < 16384) && (iy < 16384))
    {
        ix = (int16_t) (ix + ix);
        iy = (int16_t) (iy + iy);
    }
    
    // calculate ix * ix and hyp * hyp
    ixsq = (uint32_t) (ix * ix);
    ihypsq = (uint32_t) (ixsq + (iy * iy));
    
    // set result r to zero and binary search step to 16384 (0.5)
    ir = 0;
    idelta = 16384;             // 2^14 = 16384
    
    // loop over binary subdivision algorithm
    do
    {
        // generate new candidate solution for ir and test if too high or too low
        // itmp = (ir + delta)^2, range 0 to 32767^2
        itmp = (uint32_t) ((ir +idelta) * (ir + idelta));
        // itmp = (ir + delta)^2 * (ix*ix+iy*iy), range 0 to 2^31
        itmp = (itmp >> 15) * (ihypsq >> 15);
        if( itmp <= ixsq)
        {
            ir += idelta;
        }
        idelta = (int16_t)(idelta >> 1); // divide by 2
        
    } while (idelta >= MINDELTATRIG);
    
    // correct sign
    return (int16_t)(ir * isignx);
    
}

/* iHundredAtan2Deg
 * calculates 100*atan2(iy/ix) in degres for ix, iy 
 * function returns angle in degrees times 100
 */
int16_t iHundredAtan2Deg(int16_t iy, int16_t ix)
{
    int16_t iResult; 
    
    // Check for -32768 which isn't handled correctly
    if (ix == -32768)
    {
        ix = -32767;
    }
    
    if (iy == -32768)
    {
        iy = -32767;
    }
    
    // Check quadrants
    // range 0 - 90 degs
    if((ix >= 0) && (iy >= 0))
    {
        iResult = iHundredAtanDeg(iy, ix);
    }
    // range 90 to 180 degs
    else if ((ix <= 0) && (iy >= 0))
    {
        iResult = (int16_t) (18000 - (int16_t) iHundredAtanDeg(iy, (int16_t) -ix));
    }
    // range -180 to 90 degs
    else if ((ix <= 0) && (iy <=0))
    {
        iResult = (int16_t) ((int16_t) -18000 + iHundredAtanDeg((int16_t) -iy, (int16_t) -ix));
    }
    // ix >= 0, iy <= 0, range -90 to 0 degs
    else
    {
        iResult = (int16_t) (-iHundredAtanDeg((int16_t) -iy, ix));
    }
    
    return iResult;
}

/* iHundredAtanDeg
 * calculates 100 * atan(iy/ix) range 0 to 9000
 */
int16_t iHundredAtanDeg(int16_t iy, int16_t ix)
{
    int32_t iAngle;     // angle in deg * 100
    int16_t iRatio;     // ratio of iy / ix 
    int32_t iTmp;   
    
    if ((ix == 0) && (iy == 0)) 
    {
        return 0;
    }
    
    if ((ix == 0) && (iy != 0))
    {
        return 9000;
    }
    
    if (iy <= ix)
    {
        iRatio = iDivide(iy, ix); // return frac in range 0. to 1.
    }
    else
    {   
        iRatio = iDivide(ix, iy);
    }
    
    // 1st, 3rd, 5th order polynomial approx
    iAngle = (int32_t) K1 * (int32_t) iRatio;
    iTmp = ((int32_t) iRatio >> 5) * ((int32_t) iRatio >> 5) * ((int32_t) iRatio >> 5); 
    iAngle += (iTmp >> 15) * (int32_t) K2;
    iTmp = (iTmp >> 20) * ((int32_t) iRatio >> 5) * ((int32_t) iRatio >> 5);
    iAngle += (iTmp >> 15) * (int32_t) K3;
    iAngle = iAngle >> 15;
    
    // check if > 45 degs
    if (iy > ix)
    {
        iAngle = (int16_t) (9000 - iAngle);
    }
    
    // limit range
    if (iAngle < 0)
    {
        iAngle = 0;
    }
    
    if (iAngle > 9000)
    { 
        iAngle = 9000;
    }
    
    return (int16_t) iAngle;
}

/* iDivide
 * calculate ir = iy / ix with iy <= ix, and ix, iy > 0
 */
int16_t iDivide(int16_t iy, int16_t ix)
{
    int16_t itmp;
    int16_t ir;         // result = iy / ix
    int16_t idelta;
    
    // set result = 0
    ir = 0;
    idelta = 16384;
    
    // to reduce quantization effects
    while ((ix < 16384) && (iy < 16384))
    {
        ix = (int16_t) (ix + ix);
        iy = (int16_t) (iy + iy);
    }
    
    do
    {
        itmp = (int16_t) (ir + idelta);
        itmp = (int16_t) ((itmp * ix) >> 15);
        if (itmp <= iy)
        {
            ir += idelta;
        }       
        idelta = (int16_t) (idelta >> 1);
    } while (idelta >= MINDELTADIV);
    
    return ir;
}

int16_t ieCompass(int16_t magX, int16_t magY, int16_t magZ, int16_t accelX, int16_t accelY, int16_t accelZ)
{
    
    int16_t iSin, iCos;
        
    // Hard iron off setting here if done
    magX -= iVx;
    magY -= iVy;
    magZ -= iVz;
    char str[32];
    sprintf(str, "magX %d, magY %d, magZ %d\n", magX, magY, magZ);
    TRACE(str);
    
    iPhi = iHundredAtan2Deg(accelY, accelZ);

    // Calculate sin and cosine of roll angle Phi
    iSin = iTrig(accelY, accelZ);
    iCos = iTrig(accelZ, accelY);

    // De rotate by roll angle Phi
    iBfy = (int16_t)    ((magY * iCos - magZ * iSin) >> 15);
    magZ = (int16_t)    ((magY * iSin + magZ * iCos) >> 15);
    accelZ = (int16_t)  ((accelY * iSin + accelZ * iCos) >> 15);

    // Calculate pitch angle Theta
    iThe = iHundredAtan2Deg((int16_t) -accelX, accelZ);

    // restrict pitch angle
    if(iThe > 9000)
    {
        iThe = (int16_t) (18000 - iThe);
    } 
    if(iThe < -9000)
    {
        iThe = (int16_t) (-18000 - iThe);
    }

    // Calculate sin and cosine of Theta
    iSin = (int16_t) -iTrig(accelX, accelZ);
    iCos = iTrig(accelZ, accelX);

    // Correct cos if pitch in range
    if (iCos < 0)
    {
        iCos = (int16_t) -iCos;
    }

    // de rotate by Theta
    iBfx = (int16_t) ((magX * iCos + magZ * iSin) >> 15);
    iBfz = (int16_t) ((-magX * iSin + magZ * iCos) >> 15);

    // Calculate current yaw = e-compass angle Psi
    iPsi = iHundredAtan2Deg((int16_t)-iBfy, iBfx);
    
    return (int16_t) iPsi;
            
}