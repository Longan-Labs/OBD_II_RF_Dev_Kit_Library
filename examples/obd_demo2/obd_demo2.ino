/************************************************************************************************* 
  OBD-II_PIDs TEST CODE
  Joney @ Dec 2017
  
  Modify @ 2019-7-9, add ext frame
  
  Query
  send id: 0x7df
      dta: 0x02, 0x01, PID_CODE, 0, 0, 0, 0, 0
  Response
  From id: 0x7E9 or 0x7EA or 0x7EB
      dta: len, 0x41, PID_CODE, byte0, byte1(option), byte2(option), byte3(option), byte4(option)
      
  https://en.wikipedia.org/wiki/OBD-II_PIDs
  
  Input a PID, then you will get reponse from vehicle, the input should be end with '\n'
***************************************************************************************************/
#include <Serial_CAN_Module.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <I2C_LCD.h>
I2C_LCD LCD;
uint8_t I2C_LCD_ADDRESS = 0x51; //Device address configuration, the default value is 0x51.

Serial_CAN can;

#define STANDARD_CAN_11BIT      1       // That depands on your car. some 1 some 0. 

#define can_tx  A0           // tx of serial can module connect to D2
#define can_rx  A1           // rx of serial can module connect to D3

#define PID_ENGIN_PRM       0x0C
#define PID_VEHICLE_SPEED   0x0D
#define PID_COOLANT_TEMP    0x05

#if STANDARD_CAN_11BIT
#define CAN_ID_PID          0x7DF
#else
#define CAN_ID_PID          0x18db33f1
#endif


int speed = 0;
int engin = 0;

unsigned char PID_INPUT;
unsigned char getPid    = 0;

#if STANDARD_CAN_11BIT
unsigned long mask[4] = 
{
    0, 0x7FC,                // ext, maks 0
    0, 0x7FC,                // ext, mask 1
};

unsigned long filt[12] = 
{
    0, 0x7E8,                // ext, filt 0
    0, 0x7E8,                // ext, filt 1
    0, 0x7E8,                // ext, filt 2
    0, 0x7E8,                // ext, filt 3
    0, 0x7E8,                // ext, filt 4
    0, 0x7E8,                // ext, filt 5
};

#else
unsigned long mask[4] =
{
    1, 0x1fffffff,               // ext, maks 0
    1, 0x1fffffff,
};
 
unsigned long filt[12] =
{
    1, 0x18DAF110,                // ext, filt
    1, 0x18DAF110,                // ext, filt 1
    1, 0x18DAF110,                // ext, filt 2
    1, 0x18DAF110,                // ext, filt 3
    1, 0x18DAF110,                // ext, filt 4
    1, 0x18DAF110,                // ext, filt 5
};
#endif

void set_mask_filt()
{
    /*
     * set mask, set both the mask to 0x3ff
     */

    if(can.setMask(mask))
    {
        Serial.println("set mask ok");
    }
    else
    {
        Serial.println("set mask fail");
    }
    
    /*
     * set filter, we can receive id from 0x04 ~ 0x09
     */
    if(can.setFilt(filt))
    {
        Serial.println("set filt ok");
    }
    else 
    {
        Serial.println("set filt fail");
    }
    
}

void sendPid(unsigned char __pid)
{
    unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
    Serial.print("SEND PID: 0x");
    Serial.println(__pid, HEX);

#if STANDARD_CAN_11BIT
    can.send(CAN_ID_PID, 0, 0, 8, tmp);   // SEND TO ID:0X55
#else
    can.send(CAN_ID_PID, 1, 0, 8, tmp);   // SEND TO ID:0X55
#endif

}

void disp()
{
    LCD.CharGotoXY(50,16);
    LCD.print(speed, DEC);
    LCD.print("    ");
    LCD.CharGotoXY(50,32);
    LCD.print(engin, DEC);
    LCD.print("    ");
}

void lcdInit()
{
    Wire.begin();         //I2C controller initialization.
    
    LCD.CleanAll(WHITE);    //Clean the screen with black or white.
    delay(1000);            //Delay for 1s.
    
    //8*16 font size, auto new line, black character on white back ground.
    LCD.FontModeConf(Font_8x16_1, FM_ANL_AAA, BLACK_BAC); 

    //Set the start coordinate.
    LCD.CharGotoXY(0,16);
    //Print string on I2C_LCD.
    LCD.print("Speed: ");
    LCD.CharGotoXY(0,32);
    LCD.print("Engin: ");
}

void setup()
{
    Serial.begin(9600);
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH);
    pinMode(13, OUTPUT);
    can.begin(can_tx, can_rx, 9600);      // tx, rx
    
    // set baudrate of CAN Bus to 500Kb/s
    if(can.canRate(CAN_RATE_500))
    {
        Serial.println("set can rate ok");
    }
    else
    {
        Serial.println("set can rate fail");
    }
    
    set_mask_filt();
    
    Serial.println("begin");
    lcdInit();
}


void loop()
{
    taskCanRecv();
    taskSendPid();
    taskDisp();         
    ledBlink();
}

void ledBlink()
{
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 200)return;
    timer_s = millis();
    digitalWrite(13, 1-digitalRead(13));
}

void taskDisp()
{
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 500)return;
    timer_s = millis();
    
    disp();
}

void taskSendPid()
{
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 100)return;
    timer_s = millis();
    
    static int __st = 0;
    __st = 1-__st;

    sendPid((__st == 1) ? PID_VEHICLE_SPEED : PID_ENGIN_PRM);
}

unsigned char len = 0;
unsigned long id  = 0;
unsigned char dta[8];
    
void taskCanRecv()
{
    if(can.recv(&id, dta))                   // check if get data
    {
        if(dta[1] == 0x41)
        {
            if(dta[2] == PID_ENGIN_PRM)     // engin rpm
            {
                engin = (256.0*(float)dta[3]+(float)dta[4])/4.0;
            }
            else if(dta[2] == PID_VEHICLE_SPEED)    // vehicle speed
            {
                speed = dta[3];
            }
        }
    }
}

// END FILE