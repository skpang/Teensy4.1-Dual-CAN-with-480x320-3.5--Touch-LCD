#include "lvgl.h"
#include <Wire.h>
#include <ILI9488_t3.h>
#include <Adafruit_FT6206.h>
#include <SPI.h>
#include <FlexCAN_T4.h>

#include "ui.h"
#include "ui_helpers.h"

FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> FD;
FlexCAN_T4<CAN2, RX_SIZE_64, TX_SIZE_16> can2;

elapsedMillis task1;

#define TFT_CS 14
#define TFT_DC 15

int LCD_RESET = 41;
int LCD_BL = 17;
uint16_t frame_count = 0;
bool set_can_start = 1;
bool set_can_start_old = 0;

/*Set to your screen resolution*/
#define TFT_HOR_RES   480
#define TFT_VER_RES   320
/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

ILI9488_t3 tft = ILI9488_t3(&SPI, TFT_CS, TFT_DC);
Adafruit_FT6206 ts = Adafruit_FT6206();

void sendframe()
{
  CANFD_message_t msg;
  CAN_message_t msg2;
  msg2.id = 0x40;
    
  msg2.buf[0] = 0xff & frame_count; msg2.buf[1] =(0xff00 & frame_count) >> 8; ; msg2.buf[2] = 3; msg2.buf[3] = 4;
  msg2.buf[4] = 5; msg2.buf[5] = 6; msg2.buf[6] = 7; msg2.buf[7] = 8;
  msg2.seq = 1;
  can2.write(MB15, msg2);

  msg.len = 64;
  msg.id = 0x321;
  msg.seq = 1;
  msg.buf[0] = 0xff & frame_count; msg.buf[1] =(0xff00 & frame_count) >> 8; ; msg.buf[2] = 3; msg.buf[3] = 4;
  msg.buf[4] = 5; msg.buf[5] = 6; msg.buf[6] = 7; msg.buf[7] = 8;
  FD.write( msg);
  String frame_count_str;
  frame_count_str = String(frame_count);
  lv_label_set_text(ui_FrameCount,frame_count_str.c_str());
  
  frame_count++;
}

void canSniff20(const CAN_message_t &msg) { 
  Serial.print("T4: ");
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print(" OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print(" BUS "); Serial.print(msg.bus);
  Serial.print(" LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" REMOTE: "); Serial.print(msg.flags.remote);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" IDHIT: "); Serial.print(msg.idhit);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}

/* Display flushing */
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
   uint16_t width = (area->x2 - area->x1 +1);
   uint16_t height = (area->y2 - area->y1+1);

   tft.writeRect(area->x1, area->y1, width, height, (uint16_t *)color_p);
   lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

int oldTouchX = 0;
int oldTouchY = 0;
/*Read the touchpad*/
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
  uint16_t touchX, touchY;
 // Serial.println("touch read");
  if (ts.touched())
  {   
    // Retrieve a point  
    TS_Point p = ts.getPoint(); 

    touchX = p.y;         // Rotate the co-ordinates
    touchY = p.x;
    touchY = 320-touchY;

     if ((touchX != oldTouchX) || (touchY != oldTouchY))
     {
      /*
          Serial.print("x= ");
          Serial.print(touchX,DEC);
          Serial.print(" y= ");
          Serial.println(touchY,DEC);
        */  
          oldTouchY = touchY;
          oldTouchX = touchX;
          data->state = LV_INDEV_STATE_PR; 
          //  data->state = touched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL; 
          data->point.x = touchX;
          data->point.y = touchY;
          String coord;
          coord = " X : "+ String(touchX) + " Y : " + String(touchY);
          lv_label_set_text(ui_tpcoord,coord.c_str());
     
        }
      }else
      {
        
        data->point.x = oldTouchX;
        data->point.y = oldTouchY;
        data->state =LV_INDEV_STATE_REL;
      }
  //return 0;
}

void setup() {
    pinMode(LCD_BL,OUTPUT);
    pinMode(LCD_RESET,OUTPUT);     
    pinMode(LED_BUILTIN,OUTPUT);

    digitalWrite(LCD_BL, HIGH); 
    digitalWrite(LCD_RESET, HIGH); 
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(200);
    digitalWrite(LED_BUILTIN, LOW); 
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(200);
    digitalWrite(LED_BUILTIN, LOW); 

    delay(50); 
    digitalWrite(LCD_RESET, LOW); 
    delay(20);
    digitalWrite(LCD_RESET, HIGH); 
          
    delay(80); 
    lv_init();
    lv_tick_set_cb(millis);

    tft.begin();          /* TFT init */
    tft.setRotation(1);   /* Landscape orientation, flipped */
   // tft.invertDisplay(1); // Invert the display colour
    tft.fillScreen(ILI9488_BLUE);
    delay(1000);
    Serial.println("LVGL demo SK Pang Electronics Ltd 09/24");
    String LVGL_Arduino = "LVGL version : ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

    if (!ts.begin(40)) { 
      Serial.println("Unable to start touchscreen.");
    } 
    else { 
        Serial.println("Touchscreen started."); 
    }
    lv_display_t * disp;
    disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

    /*Initialize the (dummy) input device driver*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);
   
    ui_init();
    Serial.println( "Setup done" );
    lv_label_set_text(ui_version,LVGL_Arduino.c_str());
    
    FD.begin();
    CANFD_timings_t config;
    config.clock = CLK_24MHz;
    config.baudrate =   500000;     //500kbps
    config.baudrateFD = 2000000;    //2000kbps
    config.propdelay = 190;
    config.bus_length = 1;
    config.sample = 75;
    FD.setRegions(64);
    FD.setBaudRate(config);
    //FD.onReceive(canSniff);
    FD.setMBFilter(ACCEPT_ALL);
    FD.distribute();
    FD.mailboxStatus();

    can2.begin();
    can2.setBaudRate(500000);
    can2.setMBFilter(ACCEPT_ALL);
    can2.enableFIFO();
    can2.enableFIFOInterrupt();
    can2.distribute();
    can2.mailboxStatus();
    can2.onReceive(FIFO, canSniff20);
}

void loop() {
  CANFD_message_t msg;
  lv_timer_handler(); /* let the GUI do its work */
   
  FD.events();
  if(set_can_start != set_can_start_old)
  {
      Serial.print("can start ");
      Serial.println(set_can_start);
      set_can_start_old = set_can_start;
  }
  if (task1 >= 200)
  { 
      task1 = task1 - 200;
      if(set_can_start == 1) sendframe();
  }

  if (FD.readMB(msg) ) {
    bool prnt = 1;
    if ( prnt ) {
      Serial.print("MB: "); Serial.print(msg.mb);
      Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
      Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
      Serial.print("  EXT: "); Serial.print(msg.flags.extended );
      Serial.print("  LEN: "); Serial.print(msg.len);
      Serial.print("  BRS: "); Serial.print(msg.brs);
      Serial.print(" DATA: ");
      for ( uint8_t i = 0; i <msg.len ; i++ ) {
        Serial.print(msg.buf[i]); Serial.print(" ");
      }
      Serial.print("  TS: "); Serial.println(msg.timestamp);
    }
  }
     delay(5);
}
