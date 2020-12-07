/******************************************************************************************************
*
*                                           WIFI CONTROL DRONE V3
*
*                      
*                                   Replacing Drone Transmitter With Mobile
*
*
*                       Consider Subcribing To My YouTube Channel(https://bit.ly/2OVPERM)
********************************************************************************************************/
char auth[] = "Authcode";//Enter the auth code
char ssid[] = "SSID";//Enter mobile Hotspot name
char pass[] = "PASSWORD";//Enter mobile Hotspot password

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#define CPU_MHZ 80
#define CHANNEL_NUMBER 8  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1000  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 5 //set PPM signal output pin on the nodemcu(GPIO)
#define DEBUGPIN 4
float VREF=1.0;//change to 1.0 for esp8266 12e standalone
volatile unsigned long next;
volatile unsigned int ppm_running=1;
int ppm[CHANNEL_NUMBER];
const byte captive_portal=0;
unsigned int alivecount=0;
unsigned long time_now = 0;
# define S_L_chorder V10
# define S_L_Rtrim V11
# define S_L_Ptrim V12
# define S_L_Ttrim V13
# define S_L_Ytrim V14
# define S_L_Rlimit V15
# define S_L_Plimit V16
# define S_L_Tlimit V17
# define S_L_Ylimit V18
float input_voltage = 0.0;
float temp=0.0;
float R1=94497.0;
float R2=9820.0;
int Pitch=1500;
int Roll=1500;
int Throttle=1000;
int Yaw=1500;
int Aux1=1000;
int Aux2=1000;
int Aux3=1000;
int Aux4=1000;
int Rlimit=500;
int Plimit=500;
int Tlimit=2000;
int Ylimit=500;
int Pitch_correction=0;
int Roll_correction=0;
int Yaw_correction=0;
int Tcr=0;
String ChOrder="AETR1234";
int O_R=0;
int O_P=1;
int O_T=2;
int O_Y=3;
int O_A1=4;
int O_A2=5;
int O_A3=6;
int O_A4=7;
WidgetTerminal terminal(V0);
WidgetLED wifirssi(V8);

//============================ PPM INTERRUPT ===============================================/
void inline ppmISR(void)
{
  static boolean state = true;

  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    next = next + (PULSE_LENGTH * CPU_MHZ);
    state = false;
    alivecount++;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
 
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;//
      next = next + ((FRAME_LENGTH - calc_rest) * CPU_MHZ);
      calc_rest = 0;
      digitalWrite(DEBUGPIN, !digitalRead(DEBUGPIN));
    }
    else{
      next = next + ((ppm[cur_chan_numb] - PULSE_LENGTH) * CPU_MHZ);
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }    
  }
  timer0_write(next);
}

void handleRoot()
{
   if(ppm_running==0)
  {
    noInterrupts();
    timer0_isr_init();
    timer0_attachInterrupt(ppmISR);
    next=ESP.getCycleCount()+1000;
    timer0_write(next);
    for(int i=0; i<CHANNEL_NUMBER; i++){
      ppm[i]= CHANNEL_DEFAULT_VALUE;
    }
    ppm_running=1;
    interrupts();
  }
}
//====================================CH ORDER IDENTIFYING========================================//
void channel_order()
{
  //getting location of the particular channel
  O_R=ChOrder.indexOf('A');
  O_P=ChOrder.indexOf('E');
  O_T=ChOrder.indexOf('T');
  O_Y=ChOrder.indexOf('R');
  O_A1=ChOrder.indexOf('1');
  O_A2=ChOrder.indexOf('2');
  O_A3=ChOrder.indexOf('3');
  O_A4=ChOrder.indexOf('4');
}
//=========================================MAPPING THE CHANNEL==============================//
void inline channelmap()
{
  //placing the channel in respective location
    ppm[O_R]=Roll;
    ppm[O_P]=Pitch;
    ppm[O_T]=Throttle;
    ppm[O_Y]=Yaw;
    ppm[O_A1]=Aux1;
    ppm[O_A2]=Aux2;
    ppm[O_A3]=Aux3;
    ppm[O_A4]=Aux4;
}
//============================ DATA LOG INFO IN TELEMENTRY =======================================//
void log_data()
{
  //printing the all information telementry
    terminal.println("=========LOG_INFO===========");
    terminal.println("CH_ORDER: " + ChOrder);
    int temp=map(Rlimit,0,500,0,100);
    terminal.println("Roll_Limit: " + String(temp) +"%");
    temp=map(Plimit,0,500,0,100);
    terminal.println("Pitch_Limit: " + String(temp) +"%");
    temp=map(Tlimit,1000,2000,0,100);
    terminal.println("Throttle_Limit: " + String(temp) +"%");
    temp=map(Ylimit,0,500,0,100);
    terminal.println("Yaw_Limit: " + String(temp) +"%");
    terminal.println("Roll_Trim: " + String(Roll_correction));
    terminal.println("Pitch_Trim: "+ String(Pitch_correction));
    terminal.println("Throttle_Trim: "+String(Tcr));
    terminal.println("Yaw_Trim: "+ String(Yaw_correction));
    if(Aux2<1200)
    {
      terminal.println("MODE: ANGLE");
    }
    else if(Aux2>=1200&&Aux2<1700)
    {
      terminal.println("MODE: HORIZON");
    }
    else if(Aux2>=1700)
    {
      terminal.println("MODE: ACRO");
    }
    terminal.println("WIFI RSSI: "+ String(WiFi.RSSI())+" dBm");
    terminal.println("BATTERY: "+String(input_voltage)+" V");
   
}
//====================================RESETING THE TRIM & LIMIT ======================================//
void reset_function()
{
  //reset back to zero &100%
        Roll_correction=0;Pitch_correction=0;Tcr=0;Yaw_correction=0;
        ChOrder="AETR1234";
        Rlimit=500;Plimit=500;Tlimit=2000;Ylimit=500;
        Blynk.virtualWrite(S_L_chorder,ChOrder);
        Blynk.virtualWrite(S_L_Rtrim, Roll_correction);
        Blynk.virtualWrite(S_L_Ptrim, Pitch_correction);
        Blynk.virtualWrite(S_L_Ttrim, Tcr);
        Blynk.virtualWrite(S_L_Ytrim, Yaw_correction);
        Blynk.virtualWrite(S_L_Rlimit, Rlimit);
        Blynk.virtualWrite(S_L_Plimit,Plimit);
        Blynk.virtualWrite(S_L_Tlimit, Tlimit);
        Blynk.virtualWrite(S_L_Ylimit, Ylimit);
        terminal.println("RESET_DONE");
}
//=======================================TELEMENTRY INPUT AND FUNCTION================================================//
BLYNK_WRITE(V0)
{
  String incoming=param.asStr();
  incoming.toUpperCase();
  String sub_string=incoming.substring(0,incoming.indexOf("("));
  //================================CHORDER============================//
  if(sub_string == "CHORDER" )
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    if(sub_data.length()!=8)
    {
      terminal.println("ERROR_COMMAND_MAKE_SURE_8CH");
    }
    else
    {
      String sub_copy="Q"+sub_data;
      if((sub_copy.indexOf("A")>0)&& (sub_copy.indexOf("T")>0)&&(sub_copy.indexOf("E")>0)&&(sub_copy.indexOf("R")>0)
      &&(sub_copy.indexOf("1")>0)&&(sub_copy.indexOf("2")>0)&&(sub_copy.indexOf("3")>0)&&(sub_copy.indexOf("4")>0))
      {
        ChOrder=sub_data;
        Blynk.virtualWrite(S_L_chorder,ChOrder);
         terminal.print("Channel Order is "+sub_data);  
         channel_order();    
      }
      else
      {
         terminal.println("ERROR_MAKE_SURE_ALL_INCLUDED");
      }    
    }
  }
  //=========================THROTTLE LIMIT========================//
  else if(sub_string == "THROTTLELIMIT"||sub_string == "TLIMIT"||sub_string == "T LIMIT"||sub_string == "THROTTLE LIMIT")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
    if(isDigit(sub_data[0])&&isDigit(sub_data[1]))
    {
      int limit=sub_data.toInt();
      if(limit>100||limit<0)
      {
        terminal.println("ERROR_LIMIT_IS_NOT_IN_RANGE");
      }
      else
      {
        Tlimit=(1000+(1000*(limit/100.0)));
        Blynk.virtualWrite(S_L_Tlimit, Tlimit);
        terminal.println("THROTTLE_LIMIT_FIXED "+String(limit) + "%");
      }
    }
    else
    {
      terminal.println("ERROR_LIMIT_IS_NOT_NUMBER");
    }
  }
//=========================PITCH LIMIT===================================//
  else if(sub_string == "PITCHLIMIT"||sub_string == "PLIMIT"||sub_string == "P LIMIT"||sub_string == "PITCH LIMIT"  )
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
    if(isDigit(sub_data[0])&&isDigit(sub_data[1]))
    {
      int limit=sub_data.toInt();
      if(limit>100||limit<0)
      {
        terminal.println("ERROR_LIMIT_IS_NOT_IN_RANGE");
      }
      else
      {
       Plimit=(1000*(limit/100.0));
       Plimit=Plimit/2;
       Blynk.virtualWrite(S_L_Plimit,Plimit);
       terminal.println("PITCH_LIMIT_FIXED "+String(limit) + "%");
      }
    }
    else
    {
      terminal.println("ERROR_LIMIT_IS_NOT_NUMBER");
    }
  }
//=============================ROLL LIMIT================================//
  else if(sub_string == "ROLLLIMIT"||sub_string == "RLIMIT"||sub_string == "R LIMIT"||sub_string == "RAW LIMIT"  )
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
    if(isDigit(sub_data[0])&&isDigit(sub_data[1]))
    {
      int limit=sub_data.toInt();
      if(limit>100||limit<0)
      {
        terminal.println("ERROR_LIMIT_IS_NOT_IN_RANGE");
      }
      else
      {
        Rlimit=(1000*(limit/100.0));
        Rlimit=Rlimit/2;
        Blynk.virtualWrite(S_L_Rlimit, Rlimit);
        terminal.println("ROLL_LIMIT_FIXED "+String(limit) + "%");
      }
    }
    else
    {
      terminal.println("ERROR_LIMIT_IS_NOT_NUMBER");
    }
  }
//=============================YAW LIMIT===================================//
  else if(sub_string == "YAWLIMIT"||sub_string == "YLIMIT"||sub_string == "Y LIMIT"||sub_string == "YAW LIMIT"  )
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
    if(isDigit(sub_data[0])&&isDigit(sub_data[1]))
    {
      int limit=sub_data.toInt();
      if(limit>100||limit<0)
      {
        terminal.println("ERROR_LIMIT_IS_NOT_IN_RANGE");
      }
      else
      {
        Ylimit=(1000*(limit/100.0));
        Ylimit= Ylimit/2;
        Blynk.virtualWrite(S_L_Ylimit, Ylimit);
        terminal.println("YAW_LIMIT_FIXED "+String(limit) + "%");
      }
    }
    else
    {
      terminal.println("ERROR_LIMIT_IS_NOT_NUMBER");
    }
  }
  //========================================Pitch trim increase=============================//
   else if(sub_string == "PITCHTRIMINC"||sub_string == "PTRIMINC"||sub_string == "P TRIMINC"||sub_string == "PITCH TRIMINC")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
       if(isDigit(sub_data[0]))
       {
        int trims=sub_data.toInt();
        Pitch_correction+=trims;
        Blynk.virtualWrite(S_L_Ptrim, Pitch_correction);
        terminal.println("PITCH_TRIM_IS_SET +"+String(trims) );
        terminal.println("OVER_ALL_PITCH_TRIM "+String(Pitch_correction) );
        Blynk.syncVirtual(V2);
       }
       else
       {
        terminal.println("ERROR_COMMAND_CHECK_DIGIT");
       }  
  }
  //========================================Pitch trim decrease=============================//
   else if(sub_string == "PITCHTRIMDEC"||sub_string == "PTRIMDEC"||sub_string == "P TRIMDEC"||sub_string == "PITCH TRIMDEC")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
       if(isDigit(sub_data[0]))
       {
        int trims=sub_data.toInt();
        Pitch_correction-=trims;
        Blynk.virtualWrite(S_L_Ptrim, Pitch_correction);
        terminal.println("PITCH_TRIM_IS_SET -"+String(trims) );
        terminal.println("OVER_ALL_PITCH_TRIM "+String(Pitch_correction) );
        Blynk.syncVirtual(V2);
       }
       else
       {
        terminal.println("ERROR_COMMAND_CHECK_DIGIT");
       }  
  }
  //========================================roll trim increase=============================//
   else if(sub_string == "ROLLTRIMINC"||sub_string == "RTRIMINC"||sub_string == "R TRIMINC"||sub_string == "ROLL TRIMINC")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
       if(isDigit(sub_data[0]))
       {
        int trims=sub_data.toInt();
        Roll_correction+=trims;
        Blynk.virtualWrite(S_L_Rtrim, Roll_correction);
        terminal.println("ROLL_TRIM_IS_SET +"+String(trims) );
        terminal.println("OVER_ALL_ROLL_TRIM "+String(Roll_correction) );
        Blynk.syncVirtual(V1);
       }
       else
       {
        terminal.println("ERROR_COMMAND_CHECK_DIGIT");
       }  
  }
 
  //========================================roll trim decrease=============================//
   else if(sub_string == "ROLLTRIMDEC"||sub_string == "RTRIMDEC"||sub_string == "R TRIMDEC"||sub_string == "ROLL TRIMDEC")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
       if(isDigit(sub_data[0]))
       {
        int trims=sub_data.toInt();
        Roll_correction-=trims;
        Blynk.virtualWrite(S_L_Rtrim, Roll_correction);
        terminal.println("ROLL_TRIM_IS_SET -"+String(trims) );
        terminal.println("OVER_ALL_ROLL_TRIM "+String(Roll_correction) );
        Blynk.syncVirtual(V1);
       }
       else
       {
        terminal.println("ERROR_COMMAND_CHECK_DIGIT");
       }  
  }

   //========================================yaw trim increase=============================//
   else if(sub_string == "YAWTRIMINC"||sub_string == "YTRIMINC"||sub_string == "Y TRIMINC"||sub_string == "YAW TRIMINC")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
       if(isDigit(sub_data[0]))
       {
        int trims=sub_data.toInt();
        Yaw_correction+=trims;
        Blynk.virtualWrite(S_L_Ytrim, Yaw_correction);
        terminal.println("YAW_TRIM_IS_SET +"+String(trims) );
        terminal.println("OVER_ALL_YAW_TRIM "+String(Yaw_correction) );
        Blynk.syncVirtual(V4);
        Blynk.syncVirtual(V5);
       }
       else
       {
        terminal.println("ERROR_COMMAND_CHECK_DIGIT");
       }  
  }
 
  //========================================yaw trim decrease=============================//
   else if(sub_string == "YAWTRIMDEC"||sub_string == "YTRIMDEC"||sub_string == "Y TRIMDEC"||sub_string == "YAW TRIMDEC")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
       if(isDigit(sub_data[0]))
       {
        int trims=sub_data.toInt();
        Yaw_correction-=trims;
        Blynk.virtualWrite(S_L_Ytrim, Yaw_correction);
        terminal.println("YAW_TRIM_IS_SET -"+String(trims) );
        terminal.println("OVER_ALL_YAW_TRIM "+String(Yaw_correction) );
       }
       else
       {
        terminal.println("ERROR_COMMAND_CHECK_DIGIT");
       }  
  }   //========================================throttle trim increase=============================//
   else if(sub_string == "THROTTLETRIMINC"||sub_string == "TTRIMINC"||sub_string == "T TRIMINC"||sub_string == "THROTTLE TRIMINC")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
       if(isDigit(sub_data[0]))
       {
        int trims=sub_data.toInt();
        Tcr+=trims;
        Blynk.virtualWrite(S_L_Ttrim, Tcr);
        terminal.println("THROTTLE_TRIM_IS_SET +"+String(trims) );
        terminal.println("OVER_ALL_THR_TRIM "+String(Tcr) );
        Blynk.syncVirtual(V3);
       }
       else
       {
        terminal.println("ERROR_COMMAND_CHECK_DIGIT");
       }  
  }
 
  //========================================throttle trim decrease=============================//
   else if(sub_string == "THROTTLETRIMDEC"||sub_string == "TTRIMDEC"||sub_string == "T TRIMDEC"||sub_string == "THROTTLE TRIMDEC")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
    //sub_data.toInt();
       if(isDigit(sub_data[0]))
       {
        int trims=sub_data.toInt();
        Tcr-=trims;
        Blynk.virtualWrite(S_L_Ttrim, Tcr);
        terminal.println("THROTTLE_TRIM_IS_SET -"+String(trims) );
        terminal.println("OVER_ALL_THR_TRIM "+String(Tcr) );
        Blynk.syncVirtual(V3);
       }
       else
       {
        terminal.println("ERROR_COMMAND_CHECK_DIGIT");
       }  
  }

  //============================MODE====================================//
   else if(sub_string == "MODE")
  {
    String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
   
       if(sub_data=="ANGLE")
       {
        Aux2=1000;
        terminal.println("MODE_SET_AS_ANGLE");
       }
       else if(sub_data=="HORIZON")
       {
        Aux2=1500;
        terminal.println("MODE_SET_AS_HORIZON");
       }
       else if(sub_data=="ACRO")
       {
        Aux2=2000;
        terminal.println("MODE_SET_AS_ACRO");
       }
       else
       {
         terminal.println("ERROR_COMMAND_CHECK_MODE");
       }
       
  }
  //===============================clear============================//
  else if(sub_string == "CLEAR")
  {
    terminal.clear();
    terminal.println(F("    ______        _______"));
    terminal.println(F("   // ^ ^//      // ^ -//"));
    terminal.println(F("  //__0_// (^_^)//__~_//"));
    terminal.println(F(" //   \\        //  \\"));
    terminal.println(F("//     \\ (^_-)//    \\"));
    terminal.println(F("      𝓡𝓮𝓷𝓰𝓪_𝓻𝓲𝓭𝓮𝓻"));
  }
  //===============================log============================//
  else if(sub_string == "LOG")
  {
    log_data();
  }

   //===============================Calibrate============================//
  else if(sub_string == "CALIBRATE"||sub_string == "CALIBRATION")
  {
    if(Aux1<1500)
    {
    Throttle=2000;
    Yaw=1000;
    Pitch=1000;
    channelmap();
    delay(1500);
    terminal.println("CALIBRATION_DONE");
    Blynk.syncVirtual(V1);
    Blynk.syncVirtual(V2);
    Blynk.syncVirtual(V3);
    Blynk.syncVirtual(V4);
    Blynk.syncVirtual(V5);
    }
    else
    {
     terminal.println("ERROR_DISARM_THE_DRONE ");
     terminal.println("AND_RETRY_CALIBRATION");
    }
  }
  //==================CORRECTION=========//
  else if(sub_string == "CORRECT"||sub_string == "CORRECTION")
  {
    if(Aux1<1500)
    {
      String sub_data=(incoming.substring(incoming.indexOf("(")+1,incoming.indexOf(")")));
      if(sub_data=="FORWARD"||sub_data=="PITCHDEC"||sub_data=="PITCH DEC")
      {
         Throttle=2000;
         Pitch=1000;
         channelmap();
         delay(1500);
         terminal.println("FORWARD_CORRECTION_DONE");
         terminal.println("RETRY_UNTIL_DRONE_STOP_DRIFT_FORWARD");
         Blynk.syncVirtual(V2);
         Blynk.syncVirtual(V3);
   
      }
      else if(sub_data=="BACKWARD"||sub_data=="PITCHINC"||sub_data=="PITCH INC")
      {
         Throttle=2000;
         Pitch=2000;
         channelmap();
         delay(1500);
         terminal.println("BACKWARD_CORRECTION_DONE");
         terminal.println("RETRY_UNTIL_DRONE_STOP_DRIFT_BACKWARD");
         Blynk.syncVirtual(V2);
         Blynk.syncVirtual(V3);
      }
       else if(sub_data=="LEFT"||sub_data=="ROLLINC"||sub_data=="ROLL INC")
      {
         Throttle=2000;
         Roll=2000;
         channelmap();
         delay(1500);
         terminal.println("LEFT_CORRECTION_DONE");
         terminal.println("RETRY_UNTIL_DRONE_STOP_DRIFT_LEFT_SIDE");
         Blynk.syncVirtual(V1);
         Blynk.syncVirtual(V3);
      }
       else if(sub_data=="RIGHT"||sub_data=="ROLLDEC"||sub_data=="ROLL DEC")
      {
         Throttle=2000;
         Roll=1000;
         channelmap();
         delay(1500);
         terminal.println("RIGHT_CORRECTION_DONE");
         terminal.println("RETRY_UNTIL_DRONE_STOP_DRIFT_RIGHT_SIDE");
         Blynk.syncVirtual(V1);
         Blynk.syncVirtual(V3);
      }
     
    }
    else
    {
      terminal.println("ERROR_DISARM_THE_DRONE ");
      terminal.println("AND_RETRY_CORRECTION");
    }
  }
  else if(sub_string == "RESET")
  {
      reset_function();
  }
 
  else
  {
     terminal.println("ERROR_COMMAND_CHECK_COMMAND");
  }
  terminal.flush();
}
//===========================================JOYSTICK DATA=================================================//
BLYNK_WRITE(V1) //ROLL
{
 int temp=param.asInt();
 
 Roll=map(temp,0,100,(1500-Rlimit),(1500+Rlimit));
 Roll=Roll+Roll_correction;
 
}
BLYNK_WRITE(V2) //PITCH
{
 int temp=param.asInt();
 Pitch=map(temp,0,100,(1500-Plimit),(1500+Plimit));
 Pitch=Pitch+Pitch_correction;
}

//============================================SLIDER================================================//
BLYNK_WRITE(V3) //THROTTLE
{
 int temp=param.asInt();
 Throttle=map(temp,0,100,1000,Tlimit);
 Throttle=Throttle+Tcr;
}
//=============================================YAW BUTTON=============================================//
BLYNK_WRITE(V4)//YAW
{
  int a=param.asInt();
  if(a==HIGH)
  {
     Yaw=1500-Ylimit;
     Yaw=Yaw+Yaw_correction;
  }
  else
  {
    Yaw=1500;
    Yaw=Yaw+Yaw_correction;
  }
}
BLYNK_WRITE(V5)//YAW
{
  int a=param.asInt();
 if(a==HIGH)
  {
     Yaw=1500+Ylimit;
     Yaw=Yaw+Yaw_correction;
  }
  else
  {
    Yaw=1500;
    Yaw=Yaw+Yaw_correction;
  }
}
//====================================ARM/DISARM===========================================//
BLYNK_WRITE(V6) //AUX1
{
 Aux1=param.asInt();
}
//*********************************STORING TRIM , LIMIT  & CHORDER IN CLOUD *******************************//
BLYNK_WRITE(S_L_chorder)
{
  //restoring int value
  ChOrder = param.asString();
}
BLYNK_WRITE(S_L_Rtrim)
{
  //restoring int value
  Roll_correction = param.asInt();
}
BLYNK_WRITE(S_L_Ptrim)
{
  //restoring int value
  Pitch_correction = param.asInt();
}
BLYNK_WRITE(S_L_Ttrim)
{
  //restoring int value
  Tcr = param.asInt();
}
BLYNK_WRITE(S_L_Ytrim)
{
  //restoring int value
  Yaw_correction = param.asInt();
}
BLYNK_WRITE(S_L_Rlimit)
{
  //restoring int value
  Rlimit = param.asInt();
}
BLYNK_WRITE(S_L_Plimit)
{
  //restoring int value
  Plimit = param.asInt();
}
BLYNK_WRITE(S_L_Tlimit)
{
  //restoring int value
  Tlimit = param.asInt();
}
BLYNK_WRITE(S_L_Ylimit)
{
  //restoring int value
  Ylimit = param.asInt();
}
//*********************************************************************************//
//=====================================SIGNAL STRENGTH CALCULATION==========================================//
void signalrange()
{
  int a=(WiFi.RSSI())*(-1);
  if(a>=30 && a<60)
  {
     Blynk.setProperty(V8,"color","#00ff00");
  }
  else if(a>=60 && a<70)
  {
     Blynk.setProperty(V8,"color","#ffff00");
  }
  else if(a>=70 && a<80)
  {
     Blynk.setProperty(V8,"color","#ff8000");
  }
  else if(a>=80)
  {
     Blynk.setProperty(V8,"color","#ff0000");
  }
}
//===================================ADC=================================//
void voltage_measure()
{
 int v= analogRead(A0);
 int batt=0;
 int analog_value = analogRead(A0);
 temp = (analog_value*VREF) / 1024.0;
 input_voltage = temp / (R2/(R1+R2));
 Blynk.virtualWrite(V7,input_voltage);
}
//============================SETUP===================================//
void setup()
{  
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  Blynk.syncAll();
  terminal.clear();
  wifirssi.on();
  pinMode(sigPin,OUTPUT);
  digitalWrite(sigPin, !onState); //set the PPM signal pin to the default state (off)
  pinMode(DEBUGPIN,OUTPUT);
  digitalWrite(DEBUGPIN, !onState); //set the PPM signal pin to the default state (off)
  ppm[0]=Roll;
  ppm[1]=Pitch;
  ppm[2]=Throttle;
  ppm[3]=Yaw;
  ppm[4]=Aux1;
  ppm[5]=Aux2;
  ppm[6]=Aux3;
  ppm[7]=Aux4;
  channel_order();
  noInterrupts();
  timer0_detachInterrupt();
  ppm_running=0;
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(ppmISR);
  next=ESP.getCycleCount()+1000;
  timer0_write(next);
  channel_order();
  channelmap();
  interrupts();
  terminal.clear();
  terminal.println(F("    ______        _______"));
  terminal.println(F("   // ^ ^//      // ^ -//"));
  terminal.println(F("  //__0_// (^_^)//__~_//"));
  terminal.println(F(" //   \\        //  \\"));
  terminal.println(F("//     \\ (^_-)//    \\"));
  terminal.println(F("      𝓡𝓮𝓷𝓰𝓪_𝓻𝓲𝓭𝓮𝓻"));
  terminal.flush();
  Serial.println(F("    ______        _______"));
  Serial.println(F("   // ^ ^//      // ^ -//"));
  Serial.println(F("  //__0_// (^_^)//__~_//"));
  Serial.println(F(" //   \\        //  \\"));
  Serial.println(F("//     \\ (^_-)//    \\"));
  Serial.println(F("      𝓡𝓮𝓷𝓰𝓪_𝓻𝓲𝓭𝓮𝓻"));
}
//======================================LOOP=====================================//
void loop()
{
  Blynk.run();
  channelmap();
  signalrange();
  voltage_measure();
}
