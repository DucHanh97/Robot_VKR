
#include <OneButton.h>
#include <SoftwareSerial.h>
#include <stdio.h>
#include <string.h>

int BT1    = 2;
int BT2    = 4;
int ANL_X1 = A0;
int ANL_Y1 = A1;
int ANL_Y2 = A2;
int ANL_X2 = A3;
int TX_PIN = 13;
int RX_PIN = 12;
int LED    = 3;
int S1     = 11;
int S2     = 10;
int S3     = 9;
int S4     = 5;
int TX     = 13;
int RX     = 12;

int adc_x1;
int adc_y1;
int adc_x2;
int adc_y2;
int bt1;
int bt2;

OneButton Button_1(BT1, true);
OneButton Button_2(BT2, true);

SoftwareSerial mySerial = SoftwareSerial(RX, TX);

char *convetor_adc_x(int adc_value)
{
  if(adc_value < 94)
  {
    return "-5";
  }
  else if(adc_value < 187)
  {
    return "-4";
  }
  if(adc_value < 280)
  {
    return "-3";
  }
  if(adc_value < 373)
  {
    return "-2";
  }
  if(adc_value < 466)
  {
    return "-1";
  }
  if(adc_value < 559)
  {
    return "0";
  }
  if(adc_value < 652)
  {
    return "1";
  }
  if(adc_value < 745)
  {
    return "2";
  }
  if(adc_value < 838)
  {
    return "3";
  }
  if(adc_value < 931)
  {
    return "4";
  }
  if(adc_value < 1024)
  {
    return "5";
  }
}

char *convetor_adc_y(int adc_value)
{
  if(adc_value < 94)
  {
    return "5";
  }
  else if(adc_value < 187)
  {
    return "4";
  }
  if(adc_value < 280)
  {
    return "3";
  }
  if(adc_value < 373)
  {
    return "2";
  }
  if(adc_value < 466)
  {
    return "1";
  }
  if(adc_value < 559)
  {
    return "0";
  }
  if(adc_value < 652)
  {
    return "-1";
  }
  if(adc_value < 745)
  {
    return "-2";
  }
  if(adc_value < 838)
  {
    return "-3";
  }
  if(adc_value < 931)
  {
    return "-4";
  }
  if(adc_value < 1024)
  {
    return "-5";
  }
}
//Z:-5,X:-5,Y:-5,K:-5,B1:1,B2:1

typedef enum
{
  STOP_MODE_STATE,
  REMOTE_MODE_STATE,
  AUTO_MODE_STATE
}RobotState;

RobotState robot_state = STOP_MODE_STATE;
char rx_data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(LED, OUTPUT);
  pinMode(BT1, INPUT_PULLUP);
  pinMode(BT2, INPUT_PULLUP);

  //setup software serial
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  mySerial.begin(115200);  

  //setup button
  Button_1.attachClick(one_click_bt1);
  Button_1.attachDoubleClick(double_click_bt1);
  Button_1.attachLongPressStart(long_press_bt1);
  Button_1.attachLongPressStop(release_button_bt1);
  Button_1.attachMultiClick(multi_click_bt1);

  Button_2.attachClick(one_click_bt2);
  Button_2.attachDoubleClick(double_click_bt2);
  Button_2.attachLongPressStart(long_press_bt2);
  Button_2.attachLongPressStop(release_button_bt2);
  Button_2.attachMultiClick(multi_click_bt2);
}

static unsigned long time_now = 0;
void loop() {
  // put your main code here, to run repeatedly:
  Button_1.tick();
  Button_2.tick();
  while (mySerial.available() > 0)
  {
      rx_data = mySerial.read();
      // mySerial.write(rx_data);
  }
  switch (rx_data)
  {
    case '0':
    {
      robot_state = STOP_MODE_STATE;
      break;
    }    
    case '1':
    {
      robot_state = REMOTE_MODE_STATE;
      break;
    }
    case '2':
    {
      robot_state = AUTO_MODE_STATE;
      break;
    }
    default:
      break;
  }
  switch (robot_state)
  {
    case STOP_MODE_STATE:
      break;
    case AUTO_MODE_STATE:
    {
      
      break;      
    }
    case REMOTE_MODE_STATE:
    {
      if(millis() - time_now >= 50)
      {
        time_now = millis();
        adc_x1 = analogRead(ANL_X1);
        adc_y1 = analogRead(ANL_Y1);
        adc_x2 = analogRead(ANL_X2);
        adc_y2 = analogRead(ANL_Y2);
        
        char cmd[30];
        sprintf(cmd, "X:%s,Y:%s,I:%s,K:%s\n", convetor_adc_x(adc_x1), convetor_adc_y(adc_y1), convetor_adc_x(adc_x2), convetor_adc_y(adc_y2));
        // Serial.print(cmd);
        
        mySerial.write(cmd);

        // while(mySerial.available() > 0)
        // {
        //   char c = mySerial.read();
        //   Serial.println(c);
        // }
      }
      break;
    }
  }  
  
}

/*>>>>>  BUTTON 1  <<<<<*/
void one_click_bt1()
{
  mySerial.write("A:1\n");
}
void double_click_bt1()
{
  mySerial.write("A:2\n");
}
void long_press_bt1()
{
  mySerial.write("A:3\n");
}
void release_button_bt1()
{
  mySerial.write("A:4\n");
}
void multi_click_bt1()
{
  mySerial.write("A:5\n");
}

/*>>>>>  BUTTON 2  <<<<<*/
void one_click_bt2()
{
  mySerial.write("B:1\n");
}
void double_click_bt2()
{
  mySerial.write("B:2\n");
}
void long_press_bt2()
{
  mySerial.write("B:3\n");
}
void release_button_bt2()
{
  mySerial.write("B:4\n");
}
void multi_click_bt2()
{
  mySerial.write("B:5\n");
}