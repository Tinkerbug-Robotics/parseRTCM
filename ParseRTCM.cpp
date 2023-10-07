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

#include <ParseRTCM.h>

// Constructors 
PARSERTCM::PARSERTCM()
{
    
    in_message = false;
    msg_indx = 0;
    last_byte = 0;

}

// Public Methods

// Return the latest latitude
double PARSERTCM::getLatitude()
{
    return data_struct.latitude;
}

// Return the latest latitude
double PARSERTCM::getLongitude()
{
    return data_struct.longitude;
}

// Provide an array of bytes from your GNSS receiver's correction data stream.
// The array may include all the data your receiver sends, or a portion of that data
// spread over multiple calls to the function. This is useful if the data is broken
// up and sent over a size limited protocol such as LoRa.
void PARSERTCM::ReadData(char *rtcm_data, unsigned int data_length)
{
    // Loop through received data, may contain more than one message
    for(int i=0;i<data_length;i++)
    {
    
        // Read the next character in the message
        char in_byte = rtcm_data[i];
    
        // Look for the start of a message
        if (!in_message && last_byte == 0xD3 && (in_byte & 0xFC) == 0x00)
        {
    
            // Set he first fields in the message
            rtcm_msg[0] = last_byte;
            rtcm_msg[1] = in_byte;
            rtcm_msg[2] = rtcm_data[i+1];
            rtcm_msg[3] = rtcm_data[i+2];
            rtcm_msg[4] = rtcm_data[i+3];
    
            // Calculte the message length and number
            msg_length = ((rtcm_msg[1] & 3) << 8) + (rtcm_msg[2] << 0) + 6;
            msg_num = (rtcm_msg[3] << 4) + (rtcm_msg[4] >> 4);

            // Increment indices
            msg_indx = 5;
            i = i+3;
    
            in_message = true;
        }
    
        // End of a message (inside a packet)
        else if (msg_indx == msg_length-1)
        {
    
            // Add last character to RTCM message
            rtcm_msg[msg_indx]=in_byte;
        
            // Send message to GPS receiver
            if(Crc24Quick(0x000000, msg_length, rtcm_msg)==0)
            {

                // Parse RTCM message
                if(msg_num == 1005)
                {
        
                    // Base station position (ECEF, m)
                    
                    // Start at 24
                    uint16_t msg_num2 = extractNextXBits(rtcm_msg,24,12);
                    // Start at 36
                    uint16_t station_id = extractNextXBits(rtcm_msg,36,12);
                    // Start at 48
                    uint8_t itrf = extractNextXBits(rtcm_msg,48,6);
                    // Start at 58
                    int64_t station_pos_int[3];
                    // Start at 58                        
                    station_pos_int[0] = convert38BitIntTo64BitInt(extractNextXBits(rtcm_msg,58,38));
                    // Start at 98
                    station_pos_int[1] = convert38BitIntTo64BitInt(extractNextXBits(rtcm_msg,98,38));
                    // Start at 138
                    station_pos_int[2] = convert38BitIntTo64BitInt(extractNextXBits(rtcm_msg,138,38));
                    
                    // ECEF station position in meters
                    for (int i=0;i<3;i++)
                    {
                        data_struct.ecef[i] = (float)station_pos_int[i]*0.0001;
                    }

                    GeodeticPoint lat_lon = ecefToGeodetic(data_struct.ecef);
                    data_struct.latitude = lat_lon.latitude;
                    data_struct.longitude = lat_lon.longitude;
                    
                }
                else if (msg_num == 1033)
                {
                  
                    char antenna_description[32]="";
                    char antenna_serial_number[32]="";
                    char receiver_type[32]="";
                    char receiver_firmware_version[32]="";
                    char receiver_serial_number[32]="";
                    
                    int N,M,I,J,K;
                    int i, j;
        
                    uint16_t msg_num2 = extractNextXBits(rtcm_msg,24,12);
                    uint16_t staid = extractNextXBits(rtcm_msg,36,12);
                    N =extractNextXBits(rtcm_msg,48,8);
                    M =extractNextXBits(rtcm_msg,64+8*N,8);
                    I=extractNextXBits(rtcm_msg,72+8*(N+M),8);
                    J=extractNextXBits(rtcm_msg,80+8*(N+M+I),8);
                    K=extractNextXBits(rtcm_msg,88+8*(N+M+I+J),8);
                    
                    i = 56;
                    
                    // Antenna Description
                    for (j=0; j<N && j<31; j++) 
                    {
                        antenna_description[j]=(char)extractNextXBits(rtcm_msg,i,8);
                        i+=8;
                    }
    
                    // Setup ID
                    int setup_ID = extractNextXBits(rtcm_msg,i, 8);
                    i+=8+8;
    
                    // Antenna Serial Number
                    for (j=0; j<M && j<31; j++)
                    {
                        antenna_serial_number[j]=(char)extractNextXBits(rtcm_msg,i,8);
                        i+=8;
                    }
                    
                    // Receiver Type
                    i+=8;
                    for (j=0; j<I && j<31; j++) 
                    {
                        receiver_type[j]=(char)extractNextXBits(rtcm_msg,i,8);
                        i+=8;
                    }
    
                    // Receiver Firmware Version
                    i+=8;
                    for (j=0; j<J && j<31; j++)
                    {
                        receiver_firmware_version[j]=(char)extractNextXBits(rtcm_msg,i,8);
                        i+=8;
                    }
    
                    // Receiver Serial Number
                    i+=8;
                    for (j=0; j<K && j<31; j++)
                    {
                        receiver_serial_number[j]=(char)extractNextXBits(rtcm_msg,i,8);
                        i+=8;
                    }
    
                }
                // Decode MSM4 type messages
                else if (msg_num == 1074 || msg_num == 1094 || msg_num == 1124)
                {
    
                    int i=24;
                    int day_of_week;
                    double time_of_day, time_of_week;
                    uint16_t msg_num2 = extractNextXBits(rtcm_msg,24,12);
    
                    // GLOSNASS
                    if (msg_num2 == 1084) 
                    {
                        day_of_week = extractNextXBits(rtcm_msg,i, 3);
                        i+= 3;
                        time_of_day = (double)extractNextXBits(rtcm_msg,i,27)*0.001;
                        i+=27;
                        time_of_week = (double)(86400*day_of_week) + time_of_day;
                     }
                     // BeiDou
                     else if (msg_num2 == 1124)
                     {
                         time_of_week = (double)extractNextXBits(rtcm_msg,i,30)*0.001;
                         i+=30;
                         time_of_week += 14.0; // BDT -> GPST
                     }
                     // GPS and Galileo
                     else if ( msg_num2 == 1074 || msg_num == 1094)
                     {
                        time_of_week = (double)extractNextXBits(rtcm_msg,i,30)*0.001;
                        i+=30;
                     }
                }
            }
            else
            {
                Serial.print("Failed CRC; Message ");Serial.print(msg_num);
                Serial.print(" of length ");Serial.print(msg_length);
                Serial.print(" CRC24 is ");Serial.println(Crc24Quick(0x000000, msg_length, rtcm_msg));
            }
    
            // Done with message
            in_message = false;
    
            msg_length = 0;
            msg_indx = 0;
                
        }
        // Reading a message
        else if (in_message)
        {
            // Add last character to RTCM message
            rtcm_msg[msg_indx]=in_byte;
            
            msg_indx++;
        }
        
        last_byte = in_byte;
    }

}

// Private methods

uint32_t PARSERTCM::Crc24Quick(uint32_t Crc, uint32_t Size, uint8_t *Buffer)
{
  static const uint32_t crctab[] = {
    0x00000000,0x01864CFB,0x038AD50D,0x020C99F6,0x0793E6E1,0x0615AA1A,0x041933EC,0x059F7F17,
    0x0FA18139,0x0E27CDC2,0x0C2B5434,0x0DAD18CF,0x083267D8,0x09B42B23,0x0BB8B2D5,0x0A3EFE2E };

  while(Size--)
  {
    Crc ^= (uint32_t)*Buffer++ << 16;
    Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
    Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
  }

  return(Crc & 0xFFFFFF);
}


// Function to extract the next X bits from the array of bytes
uint64_t PARSERTCM::extractNextXBits(uint8_t *data, uint32_t bitIndex, uint8_t numBits) 
{
    uint64_t result = 0;
    uint32_t byteIndex = bitIndex / 8;
    uint8_t bitOffset = bitIndex % 8;
  
    // Extract the bits from the array
    for (uint8_t i = 0; i < numBits; i++) 
    {
        if (bitOffset == 8) 
        {
            byteIndex++;
            bitOffset = 0;
        }
        uint8_t bit = (data[byteIndex] >> (7 - bitOffset)) & 0x01;
        result = (result << 1) | bit;
        bitOffset++;
    }

    return result;
}

int64_t PARSERTCM::convert38BitIntTo64BitInt(int64_t value_38_bit) 
{

    // Extract sign bit
    int32_t sign_bit = value_38_bit >> 37;

    // Check if the leftmost bit (MSB) is set (sign bit for 2's complement representation)
    if (sign_bit) 
    {
        // Find the 2's compliment of the 38 bit integer, this is the correct positive value
        // but it only occupies the 38 LSBs and leaves extraneous '1' bits in the MSBs.
        // Next mask out the extraneous MSBs leaving a correct, but positive 64 bit int.
        // Finally take the 2's compliment of the correct 64 bit int to get the correct
        // 64 bit negative valued int.
        value_38_bit = ~((~value_38_bit+1) & 0x3FFFFFFFFF)+1;
    }
    
    return value_38_bit;
}

void PARSERTCM::printBits(const unsigned char *buff, int len)
{
    for (int i=0;i<len;i++)
    {
        for (int j=7;j>=0;j--)
        {
            bool b = bitRead(buff[i], j);
            Serial.print(b);
        }
    }
    Serial.println("");
}

GeodeticPoint PARSERTCM::ecefToGeodetic(double ecef[3]) 
{
    GeodeticPoint geodetic;

    double x, y, z;
    double latitude, longitude, altitude;

    // The variables below correspond to symbols used in the paper
    // "Accurate Conversion of Earth-Centered, Earth-Fixed Coordinates
    // to Geodetic Coordinates"
    double beta, C, dFdt, dt, dw, dz, F, G, H, i, k, m, n, p, P, t, u, v, w;

    // Intermediate variables
    double j, ww, mpn, g, tt, ttt, tttt, zu, wv, invuv, da;
    double t1, t2, t3, t4, t5, t6, t7;

    x = ecef[0];
    y = ecef[1];
    z = ecef[2];
    ww = x * x + y * y;
    m = ww * WGS84_INVAA;
    n = z * z * WGS84_P1MEEDAA;
    mpn = m + n;
    p = WGS84_INV6 * (mpn - WGS84_EEEE);
    G = m * n * WGS84_EEEED4;
    H = 2 * p * p * p + G;

    C = pow(H + G + 2 * std::sqrt(H * G), WGS84_INV3) * WGS84_INVCBRT2;
    i = -WGS84_EEEED4 - 0.5 * mpn;
    P = p * p;
    beta = WGS84_INV3 * i - C - P / C;
    k = WGS84_EEEED4 * (WGS84_EEEED4 - mpn);

    // Compute left part of t
    t1 = beta * beta - k;
    t2 = std::sqrt(t1);
    t3 = t2 - 0.5 * (beta + i);
    t4 = std::sqrt(t3);

    // Compute right part of t
    t5 = 0.5 * (beta - i);

    // t5 may accidentally drop just below zero due to numeric turbulence
    // This only occurs at latitudes close to +- 45.3 degrees
    t5 = std::fabs(t5);
    t6 = std::sqrt(t5);
    t7 = (m < n) ? t6 : -t6;

    // Add left and right parts
    t = t4 + t7;

    // Use Newton-Raphson's method to compute t correction
    j = WGS84_EED2 * (m - n);
    g = 2 * j;
    tt = t * t;
    ttt = tt * t;
    tttt = tt * tt;
    F = tttt + 2 * i * tt + g * t + k;
    dFdt = 4 * ttt + 4 * i * t + g;
    dt = -F / dFdt;

    // Compute latitude (range -PI/2..PI/2)
    u = t + dt + WGS84_EED2;
    v = t + dt - WGS84_EED2;
    w = std::sqrt(ww);
    zu = z * u;
    wv = w * v;
    latitude = std::atan2(zu, wv);

    // Compute altitude
    invuv = 1 / (u * v);
    dw = w - wv * invuv;
    dz = z - zu * WGS84_P1MEE * invuv;
    da = std::sqrt(dw * dw + dz * dz);
    altitude = (u < 1) ? -da : da;

    // Compute longitude (range -PI..PI)
    longitude = std::atan2(y, x);

    // Convert from radians to degrees
    geodetic.latitude = latitude * 180.0 / M_PI;
    geodetic.longitude = longitude * 180.0 / M_PI;

    return geodetic;
}
