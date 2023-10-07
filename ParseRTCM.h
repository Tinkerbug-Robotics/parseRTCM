/**********************************************************************
*
* MIT License
*
* Copyright (c) 2023 Tinkerbug Robotics
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
* 
* Authors: 
* Christian Pedersen; tinkerbug@tinkerbugrobotics.com
* 
**********************************************************************/

#ifndef PARSERTCM_h
#define PARSERTCM_h

#include <Arduino.h>

struct RTCM_Data
{
    double latitude;
    double longitude;
    double ecef[3];
};

struct GeodeticPoint
{
    double latitude;
    double longitude;
};

// ECEF to Lat/Lon/Alt parameters
#define WGS84_INVAA +2.45817225764733181057e-0014    /* 1/(a^2) */
#define WGS84_P1MEEDAA +2.44171631847341700642e-0014 /* (1-(e^2))/(a^2) */
#define WGS84_EEEE +4.48147234524044602618e-0005     /* e^4 */
#define WGS84_INVCBRT2 +7.93700525984099737380e-0001 /* 1/(2^(1/3)) */
#define WGS84_INV3 +3.33333333333333333333e-0001     /* 1/3 */
#define WGS84_INV6 +1.66666666666666666667e-0001     /* 1/6 */
#define WGS84_EEEED4 +1.12036808631011150655e-0005   /* (e^4)/4 */
#define WGS84_EED2 +3.34718999507065852867e-0003     /* (e^2)/2 */
#define WGS84_P1MEE +9.93305620009858682943e-0001    /* 1-(e^2) */

class PARSERTCM
{
    public:
    
    PARSERTCM();
  
    double getLongitude();
    double getLatitude();
    
    void ReadData(char *rtcm_data, unsigned int data_length);
    RTCM_Data data_struct;

private:

    uint64_t extractNextXBits(uint8_t *data, uint32_t bitIndex, uint8_t numBits);
    int64_t convert38BitIntTo64BitInt(int64_t value_38_bit);
    void printBits(const unsigned char *buff, int len);
    GeodeticPoint ecefToGeodetic(double ecef[3]);
    uint32_t Crc24Quick(uint32_t Crc, uint32_t Size, uint8_t *Buffer);
    
    // Variables for reading RTCM messages across data inputs
    bool in_message;
    unsigned int msg_num;
    unsigned int msg_length;
    uint8_t rtcm_msg[2500];
    unsigned int msg_indx;
    char last_byte;
};

#endif



