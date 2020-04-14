/******************************************************************************

ESP32-CAM remote image access via FTP. Take pictures with ESP32 and upload it via FTP making it accessible for the outisde network. 
Leonardo Bispo
July - 2019
https://github.com/ldab/ESP32-CAM-Picture-Sharing

Distributed as-is; no warranty is given.

******************************************************************************/
#include "Arduino.h"



// Enable Debug interface and serial prints over UART1
#define DEGUB_ESP

#include <WiFi.h>
#include <WiFiClient.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"

#include "esp_http_server.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "soc/soc.h"           //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems

#include "driver/rtc_io.h"


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


// Variables to save date and time
int hourOfDay;

// Connection timeout;
#define CON_TIMEOUT   10*1000                     // milliseconds

// Not using Deep Sleep on PCB because TPL5110 timer takes over.
#define TIME_TO_SLEEP (uint64_t)60*60*1000*1000   // microseconds

#ifdef DEGUB_ESP
  #define DBG(x) Serial.println(x)
#else 
  #define DBG(...)
#endif

// FTP Client Lib
#include "ESP32_FTPClient.h"

// Your WiFi credentials.
char ssid[] = "Chicken Coop";
char pass[] = "bledisloe";

// FTP Server credentials
char ftp_server[] = "ftp.greenbridge.net.au";
char ftp_user[]   = "parrot_cam@greenbridge.net.au";
char ftp_pass[]   = "N35a%j5TS!UG";

// Camera buffer, URL and picture name
camera_fb_t *fb = NULL;
String pic_name = "parrot";

// Variable marked with this attribute will keep its value during a deep sleep / wake cycle.
RTC_DATA_ATTR uint64_t bootCount = 0;


ESP32_FTPClient ftp (ftp_server, ftp_user, ftp_pass);

void deep_sleep(void);
void FTP_upload( void );
bool take_picture(void);

time_t prevDisplay = 0; // when the digital clock was displayed


void setup()
{
#ifdef DEGUB_ESP
  Serial.begin(115200);
  Serial.setDebugOutput(true);
#endif


  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = 5;
  config.pin_d1       = 18;
  config.pin_d2       = 19;
  config.pin_d3       = 21;
  config.pin_d4       = 36;
  config.pin_d5       = 39;
  config.pin_d6       = 34;
  config.pin_d7       = 35;
  config.pin_xclk     = 0;
  config.pin_pclk     = 22;
  config.pin_vsync    = 25;
  config.pin_href     = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn     = 32;
  config.pin_reset    = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  config.frame_size = FRAMESIZE_XGA; // set picture size, FRAMESIZE_XGA = 1024x768
  config.jpeg_quality = 10;          // quality of JPEG output. 0-63 lower means higher quality
  config.fb_count = 2;


  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.print("Camera init failed with error 0x%x");
    DBG(err);
    delay(10000);
    return;
  }

  // Change extra settings if required
  //sensor_t * s = esp_camera_sensor_get();
  //s->set_vflip(s, 0);       //flip it back
  //s->set_brightness(s, 2);  //up the blightness just a bit
  //s->set_saturation(s, -2); //lower the saturation
  
  // Enable timer wakeup for ESP32 sleep
  esp_sleep_enable_timer_wakeup( TIME_TO_SLEEP );

  WiFi.begin( ssid, pass );
  DBG("\nConnecting to WiFi");

  while ( WiFi.status() != WL_CONNECTED && millis() < CON_TIMEOUT )
  {
    delay(500);
    Serial.print(".");
  }

  if( !WiFi.isConnected() )
  {
    DBG("Failed to connect to WiFi, going to sleep");
    deep_sleep();
  }

  DBG("");
  DBG("WiFi connected");
  DBG( WiFi.localIP() );

// Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +10 = 36000

  timeClient.setTimeOffset(36000);

}

void loop()
{

    while(!timeClient.update()) {
      timeClient.forceUpdate();
    }

    hourOfDay = timeClient.getHours();


    if (hourOfDay >= 6 && hourOfDay <= 18) {
        // Take picture
      if( take_picture() )
      {
        FTP_upload();

        deep_sleep();
      }
      else
      {
        DBG("Capture failed, sleeping");
        deep_sleep();
      }
    }


  if( millis() > CON_TIMEOUT)
  {
    DBG("Back to sleep guys - Birds are in bed");

    deep_sleep();
  }

}

void deep_sleep()
{
  DBG("Going to sleep after: " + String( millis() ) + "ms");
  Serial.flush();

  esp_deep_sleep_start();
}

bool take_picture()
{
  DBG("Taking picture now");

  fb = esp_camera_fb_get();  

  if(!fb)
  {
    DBG("Camera capture failed");
    return false;
  }
  
  // Rename the picture with the time string
  // pic_name += String( now() ) + ".jpg";
  pic_name += "_" + String(hourOfDay) + ".jpg";
  DBG("Camera capture success, saved as:");
  DBG( pic_name );

  return true;
}

void FTP_upload()
{
  DBG("Uploading via FTP");
  ftp.OpenConnection();
  DBG("Connection Opened");
  //Create a file and write the image data to it;
  ftp.InitFile("Type I");
  ftp.ChangeWorkDir(""); // change it to reflect your directory
  const char *f_name = pic_name.c_str();
  ftp.NewFile( f_name );
  
  ftp.WriteData(fb->buf, fb->len);
  ftp.CloseFile();

  // Breath, withouth delay URL failed to update.
  delay(100);

}

