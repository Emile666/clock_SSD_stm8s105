/*==================================================================
  File Name    : esp8266_ntp_server.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the Arduino sketch for reading an
            NTP Time Server and delivering time and/or date via UART.
            It uses UART commands S0 (version), E0 (time) and E1 (date)
  ------------------------------------------------------------------
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with This software.  If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */ 
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <cstdio>

// Replace with your network credentials
const char *ssid     = "YourSSID";
const char *password = "YourPassword";

#define BUFLEN (50)
char rs232_buff[BUFLEN];
byte rs232_idx = 0;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print(".");
  } // while
  Serial.println("connected.");
  
  timeClient.begin(); // Initialize a NTPClient to get time
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(3600);
} // setup()

uint8_t execute_single_command(char *s)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   uint8_t  rval = 0;
   char     s2[25];
   
   switch (s[0])
   {
     case 'e': 
        if (num > 1) 
           rval = 1;
        else if (!num)
        { // e0: get date & time
          timeClient.update();
          unsigned long epochTime = timeClient.getEpochTime();
          // Get a time structure
          struct tm *ptm = gmtime ((time_t *)&epochTime); 
          sprintf(s2,"e0 %02d-%02d-%4d.",ptm->tm_mday, ptm->tm_mon+1, ptm->tm_year+1900);
          Serial.print(s2);
          Serial.println(timeClient.getFormattedTime());
        } // if
        else timeClient.begin(); // e1: init. again
        break;
        
     case 's': // s0: get version info
        if (!num)
        {
          Serial.println("esp8266_ntp_v010");
        } // if
        break;
     default: rval = 1; // ERR
              break;
   } // switch
   return rval; 
} // execute_single_command()

void loop() {
  if (Serial.available() > 0)
  {
    byte c = Serial.read();
    if ((rs232_idx < sizeof rs232_buff) && (c != '\n'))
    {
      rs232_buff[rs232_idx++] = c;
      if (c == '\r') // (CR,13) end of word
      {
        execute_single_command(rs232_buff);
        rs232_idx = 0;  
      } // if
    } // if
    else rs232_idx = 0; // reset index
  } // if
} // loop()
