


  /*
 Hexpod Project
 Author: Yongxin Guo
 Contact information: yongxin.guo@stonybrook.edu
 Last Modified: 2018/4/1
                     Coordinates for each coxa frame:        | Y-axis
                                                             |
                                                             |
                                                  ----------------------- X-axis
                                                          /  |
                                                         /   |
                                                Z-axis  /    | 
 */
#include <coordinate.h>
#include <math.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
// List of supported pins: https://www.arduino.cc/en/Reference/SoftwareSerial
// Bluetooth TX -> Arduino D8
const int BLUETOOTH_TX = 12;
// Bluetooth RX -> Arduino D7
const int BLUETOOTH_RX = 13;
int throttle, steering, prevMillis, sliderVal, button, sliderId;


#define SSC_32_RX 10
#define SSC_32_TX 11
#define LF 0              //             0---------|     |---------3
#define LM 1              //             1---------|     |---------4
#define LR 2              //             2---------|     |---------5
#define RF 3
#define RM 4 
#define RR 5

SoftwareSerial SSC32(SSC_32_RX, SSC_32_TX);
SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(bluetooth); // pass reference of bluetooth object to ArduinoCommander.



const double pi=2*acos(0.0);
const int predigit=6;
const float radToDegree=180/pi;
const float angleRotation[6]={-0.5236,-1.5708,-2.6180,0.5236,1.5708,2.6180};             //Rotation angle used in rotation matrix for six legs; Angle in rads.


//-------------------------------------------------------------------------
//======================|leg0|===|leg1|===|leg2|===|leg3|===|leg4|===|leg5|
float initial_X[6]={     -12.5,   -18.5,    -12.5,   12.5,    18.5,    12.5};              //Points on X aixs for base position, enter six X coodinates for six legs
//======================|    |===|    |===|    |===|    |===|    |===|    |
float initial_Y[6]={      17.5,    0.0,     -17.5,   17.5,    0.0,    -17.5};             //points on Y axis for base position, enter six Y coordinates for six legs
//======================|    |===|    |===|    |===|    |===|    |===|    |
float initial_Z[6]={     -13.0,   -13.0,    -13.0,   -13.0,   -13.0,  -13.0};              //points on Z axis for base position, enter six Z coordinates for six legs

//
//===================================== Triot   Gait    Legs    Coordinates============================================//



//================================================================================
float CoxaToBodyPosition_X[6]={-6.04,-7.17,-6.04, 6.04,  7.17, 6.04};        //X coordinates for six coxas
float CoxaToBodyPosition_Y[6]={10.45, 0,   -10.45, 10.45, 0,  -10.45};        //Y coordinates for six coxas
float CoxaToBodyPosition_Z[6]={0,0,0,0,0,0};        //Z coordinates for six coxas
//---------------------------Variables for IK algorithms----------------------------------------------
const float Femur=7.02;
const float Tibia=12.69;
const float Coxa=5.30;
const float FT=Femur*Tibia;
const float virtualCoxa=5.04;
const float v=sqrt(sq(Coxa)-sq(virtualCoxa));

//------------------------Change in angles when assigned to servos------------------
//float BETAposition[6]={};
//float ALPHAposition[6]={};
//float GAMMAposition[6]={};
//==============================change in divisions================================
//float BETAchangeDiv[6]={};
//float ALPHAchangeDiv[6]={};
//float GAMMAchangeDiv[6]={};
//==============================Base angles for servos=============================
//float baseBetaangles[6]={};
//float baseAlphaangles[6]={};
//float baseGammaangles[6]={};
//=============================Target angles for servos============================
//float targetBetaangles[6]={};
//float targetAlphaangles[6]={};
//float targetGammaangles[6]={};
//======================
int compensateNumber=5;
int division=8;
int delaytime=2.5;
int leftGammaOffset=5;
int rightGammaOffset=-5;

//=====================================================Instantaneous center of curvature=====================================================

coordinate ICC[1]={};

float distanceIccToLeg[6]={};
float IccStepAngles[6]={};
const int IccSteps=9;
float strideLength=5;

float initialIccFrame_X[6]={};
float initialIccFrame_Y[6]={};


float FootPoint_X[6][IccSteps]={};
float FootPoint_Y[6][IccSteps]={};
float FootPoint_Z[6][IccSteps]={};

float BETAangle[6][IccSteps]={};
float ALPHAangle[6][IccSteps]={};
float GAMMAangle[6][IccSteps]={};
float BETAangleRippleGait[6][IccSteps * 2 - (IccSteps - 1) / 2] = {};
float ALPHAangleRippleGait[6][IccSteps * 2 - (IccSteps - 1) / 2] = {};
float GAMMAangleRippleGait[6][IccSteps * 2 - (IccSteps - 1) / 2] = {};

int Check=0;

float Z_Height=-11;
float Z_Raise_TripodGait; // height of lifting for tripod gait.
float Z_Raise_RippleGait;//height of lifting for ripple gait.

int sscDelayTime=300;

int sequenceStatus;  // status value for defining the swinging/pushing direction of tripod legs.

int gaitStatus = 0; //0 means tripod gait in default, 1 means ripple gait
int walkStyle = 0;//0 means it will walk along a straight line, 1 means it will walk along a curvy line.
void setup() {
  Serial.begin(115200);
  Serial.println("< Starting... >");
  
  // Start the software serial interface
  Serial.print("\t > Starting software serial interface to SSC-32U... ");
  SSC32.begin(115200);
  SSC32.listen();                                                  // Required for the SoftwareSerial port to receive data properly. Only one SoftSerial interface can listen at any time.
  if (SSC32.isListening())
  {
    Serial.println("Port opened successfully.");
  }
  else
  {
    Serial.println("*!*!*!* Port error.");
  }
  delay(1000);
  //===========ArduinoBlue section===========================
   bluetooth.begin(115200);
   // delay just in case bluetooth module needs time to "get ready"
   delay(100);
   Serial.println("\n Bluetooth setup complete");
   prevMillis = millis();

  //=======================================================
  SSC32.print("#20 P2166");  SSC32.print("#1 P2166");  SSC32.print("#2 P1500");
  SSC32.print("#3 P2166");  SSC32.print("#4 P2166");  SSC32.print("#5 P1500"); 
  SSC32.print("#6 P2166");  SSC32.print("#7 P2166"); SSC32.print("#8 P1500");
  SSC32.print("#9 P834"); SSC32.print("#10 P834"); SSC32.print("#11 P1500"); 
  SSC32.print("#12 P834");  SSC32.print("#13 P834"); SSC32.print("#14 P1556"); 
  SSC32.print("#15 P834");  SSC32.print("#16 P834");  SSC32.print("#17 P1500"); 
  SSC32.println("T2000");
  delay(2000);
  
  SSC32.print("#20 P1699");  SSC32.print("#1 P1655");  SSC32.print("#2 P1500");
  SSC32.print("#3 P1744");  SSC32.print("#4 P1611");  SSC32.print("#5 P1500"); 
  SSC32.print("#6 P1833");  SSC32.print("#7 P1622"); SSC32.print("#8 P1500");
  SSC32.print("#9 P1211"); SSC32.print("#10 P1333"); SSC32.print("#11 P1500"); 
  SSC32.print("#12 P1244");  SSC32.print("#13 P1422"); SSC32.print("#14 P1550"); 
  SSC32.print("#15 P1289");  SSC32.print("#16 P1278");  SSC32.print("#17 P1500"); 
  SSC32.println("T1500");
  delay(1500);
}

void loop() {

/*ICC[0].X=-2000.0;
ICC[0].Y=0.0;
Z_Raise=5.5;
sequenceStatus=1;
IccAlgorithms(ICC[0].X, ICC[0].Y, sequenceStatus, Z_Raise);*/

/*ICC[0].X=0.0;
ICC[0].Y=0.0;
Z_Raise=6.5;
sequenceStatus=1; //ROTATION AROUND CCW (when ICC.X and ICC.Y both equal to 0.0).
IccAlgorithms(ICC[0].X, ICC[0].Y, sequenceStatus, Z_Raise);*/

/*ICC[0].X=0.0;
ICC[0].Y=0.0;
Z_Raise=6.5;
sequenceStatus=-1; //ROTATION AROUND CW (when ICC.X and ICC.Y both equal to 0.0).
IccAlgorithms(ICC[0].X, ICC[0].Y, sequenceStatus, Z_Raise);*/
    button = phone.getButton();
    String str = phone.getText();
 // throttle and steering values go from 0 to 99.
   

    // ID of the slider moved
    sliderId = phone.getSliderId();

    // slider value goes from 0 to 200
    sliderVal = phone.getSliderVal();

    // display button data whenever its pressed
    if (button != -1) {
        Serial.print("Button: ");
        Serial.println(button);
    }

    // display slider data when slider moves
    if (sliderId != -1) {
        Serial.print("Slider ID: ");
        Serial.print(sliderId);
        Serial.print("\tValue: ");
        Serial.println(sliderVal);
    }
     throttle = phone.getThrottle();
     steering = phone.getSteering();
     
     
    if(button == 3){
        gaitStatus = 0;//change to tripod gait
        strideLength = 5;
        }
    else if(button == 4){
        gaitStatus = 1;
        strideLength = 6;//change to ripple gait
    }
    else if(button == 5){
      walkStyle = 0;//change to straight line walking style.
    }
    else if(button == 6){
      walkStyle = 1;//change to curvy line walking style.
    }
    else{
      //do nothing
    }

    if (throttle!=49||steering!=49) {
        Serial.println("----------------------------");
        Serial.print("Throttle: ");
        Serial.println(throttle);
        Serial.print("Steering: ");
        Serial.println(steering);
        prevMillis = millis();
        sequenceStatus=1;
        Z_Raise_RippleGait = 6;
        Z_Raise_TripodGait = 4.5;
        int Joystick_X_value = steering;
        int Joystick_Y_value = throttle;
        int ICCDistance = 0;
        if (walkStyle == 0){
          ICCDistance=2000;
        }
        else if (walkStyle == 1){
          ICCDistance=35;
        }
        else {
          //do nothing
        }
        ICC[0].X=JoystickToICC_X(Joystick_X_value,Joystick_Y_value,ICCDistance);
        ICC[0].Y=JoystickToICC_Y(Joystick_X_value,Joystick_Y_value,ICCDistance);
        if(gaitStatus == 0){
          IccAlgorithms(ICC[0].X, ICC[0].Y, sequenceStatus, Z_Raise_TripodGait,strideLength);
        }
        else if(gaitStatus == 1){
          IccAlgorithmsRippleGait(ICC[0].X, ICC[0].Y, sequenceStatus, Z_Raise_RippleGait,strideLength);
        }
        else{
          //nothing yet
        }
    }
    if (str != "") {
        Serial.println(str);
    }

    // Send string from serial command line to the phone. This will alert the user.
    if (Serial.available()) {
        Serial.write("usb: ");
        String str = Serial.readString();
        phone.sendMessage(str);
        Serial.print(str);
        Serial.write('\n');
    }
      if(button == 1){
      ICC[0].X=0.0;
      ICC[0].Y=0.0;
      Z_Raise_TripodGait=6.5;
      sequenceStatus=1;//Rotation around CCW (when ICC.X and ICC.Y both equal to 0.0)
      IccAlgorithms(ICC[0].X, ICC[0].Y, sequenceStatus, Z_Raise_TripodGait,strideLength);
     }
      else if(button == 2){
      ICC[0].X=0.0;
      ICC[0].Y=0.0;
      Z_Raise_TripodGait=6.5;
      sequenceStatus=-1;//Rotation around CW (when ICC.X and ICC.Y both equal to 0.0)
      IccAlgorithms(ICC[0].X, ICC[0].Y, sequenceStatus, Z_Raise_TripodGait,strideLength);
     }
   

}





//***********************************************************TRIPOD GAIT****************************************************************************************
void coordinateTransformation(float footPointPosition_X[][IccSteps], float footPointPosition_Y[][IccSteps], float footPointPosition_Z[][IccSteps])   {
  
 //==============================================================convertBCtoLO==============================================================================================
 for (int stepnum=0;stepnum<IccSteps; stepnum++){
  for (int legnum=LF; legnum<=RR; legnum++){
    
   footPointPosition_X[legnum][stepnum]=footPointPosition_X[legnum][stepnum]-CoxaToBodyPosition_X[legnum];
   footPointPosition_Y[legnum][stepnum]=footPointPosition_Y[legnum][stepnum]-CoxaToBodyPosition_Y[legnum];
   footPointPosition_Z[legnum][stepnum]=footPointPosition_Z[legnum][stepnum]-CoxaToBodyPosition_Z[legnum];
  
    float temporary_X=footPointPosition_X[legnum][stepnum];                                                                                             //                     -----------------
    footPointPosition_X[legnum][stepnum]=footPointPosition_X[legnum][stepnum]*cos(angleRotation[legnum])-footPointPosition_Y[legnum][stepnum]*sin(angleRotation[legnum]);   //X'=X*cosT-Y*sinT              | Rotation      |
    footPointPosition_Y[legnum][stepnum]=(temporary_X)*sin(angleRotation[legnum])+footPointPosition_Y[legnum][stepnum]*cos(angleRotation[legnum]);                          //Y'=X*sinT+Y*cosT              | matrix around |
    footPointPosition_Z[legnum][stepnum]=footPointPosition_Z[legnum][stepnum];                                                                                              //Z'=Z                          | Z axis        |
                                                                                                                                               //                                                           ---------------                      
  }
 
 
 }

}

             
//================================================================IK algorithms to calculate new angles.======================================================================
void angleTransformation(float footPointPosition_X[][IccSteps], float footPointPosition_Y[][IccSteps], float footPointPosition_Z[][IccSteps]) { 
  for (int stepnum=0;stepnum<IccSteps;stepnum++){
  for (int legnum=LF;legnum<=RR;legnum++){
   float u=sq(sqrt(sq(footPointPosition_X[legnum][stepnum])+sq(footPointPosition_Y[legnum][stepnum]))-virtualCoxa);
   BETAangle[legnum][stepnum]=(acos((sq(Femur)+sq(Tibia)-u-sq(abs(footPointPosition_Z[legnum][stepnum])-v))/(2*FT)))*radToDegree;
   ALPHAangle[legnum][stepnum]=(acos((abs(footPointPosition_Z[legnum][stepnum])-v)/(sqrt(u+sq(abs(footPointPosition_Z[legnum][stepnum])-v))))+acos((u+sq(abs(footPointPosition_Z[legnum][stepnum])-v)+sq(Femur)-sq(Tibia))/(2*Femur*sqrt(u+sq(abs(footPointPosition_Z[legnum][stepnum])-v)))))*radToDegree;
   GAMMAangle[legnum][stepnum]=(atan2(footPointPosition_Y[legnum][stepnum],footPointPosition_X[legnum][stepnum]))*radToDegree;//-90
  //Serial.println(GAMMAangle[legnum][stepnum]);
  //Serial.println("===");
  //adding offset to angles
  if (legnum==0){
    BETAangle[legnum][stepnum]=(180-BETAangle[legnum][stepnum]);
    //ALPHAangle stays the same
    GAMMAangle[legnum][stepnum]=(180-GAMMAangle[legnum][stepnum]+5);
  }
  else if(legnum==1){
    BETAangle[legnum][stepnum]=(180-BETAangle[legnum][stepnum]);
    ALPHAangle[legnum][stepnum]=(ALPHAangle[legnum][stepnum]-3);
    GAMMAangle[legnum][stepnum]=(180-GAMMAangle[legnum][stepnum]+3);
  }
  else if(legnum==2){
    BETAangle[legnum][stepnum]=(180-BETAangle[legnum][stepnum]+5);
    //ALPHAangle stays the same
    GAMMAangle[legnum][stepnum]=(180-GAMMAangle[legnum][stepnum]+3);
  }
  else if(legnum==3){
    //BETAangle stays the same
    ALPHAangle[legnum][stepnum]=(180-ALPHAangle[legnum][stepnum]-3);
    GAMMAangle[legnum][stepnum]=(180-GAMMAangle[legnum][stepnum]+3);
  }
  else if(legnum==4){
    //BETAangle stays the same
    ALPHAangle[legnum][stepnum]=(180-ALPHAangle[legnum][stepnum]+7);
    GAMMAangle[legnum][stepnum]=(180-GAMMAangle[legnum][stepnum]+5);
  }
  else if(legnum==5){
    //BETAangle stays the same
    ALPHAangle[legnum][stepnum]=(180-ALPHAangle[legnum][stepnum]-5);
    GAMMAangle[legnum][stepnum]=(180-GAMMAangle[legnum][stepnum]+5);
  }

  //Serial.println(BETAangle[legnum][stepnum]);
  //Serial.println("*****");
  BETAangle[legnum][stepnum]= (BETAangle[legnum][stepnum]-30.0)*((2166.0-834.0)/(150.0-30.0))+834.0;//                   map(BETAangle[legnum][stepnum],30.0,150.0,834.0,2166.0);
  ALPHAangle[legnum][stepnum]=(ALPHAangle[legnum][stepnum]-30.0)*((2166.0-834.0)/(150.0-30.0))+834.0;  //                                      map(ALPHAangle[legnum][stepnum],30.0,150.0,834.0,2166.0);
  GAMMAangle[legnum][stepnum]=(GAMMAangle[legnum][stepnum]-30.0)*((2166.0-834.0)/(150.0-30.0))+834.0;//                   map(GAMMAangle[legnum][stepnum],30.0,150.0,834.0,2166.0);

  
  //Serial.println(BETAangle[legnum][stepnum]);
  //Serial.println("*********BETA above******");
  //Serial.println(ALPHAangle[legnum][stepnum]);
  //Serial.println("*********ALPHA above********");
  //Serial.println(GAMMAangle[legnum][stepnum]);
  //Serial.println("*********GAMMA above********");
     }
  //Serial.println(BETAangle[3][stepnum]);
  //Serial.println("*********BETA above******");
  //Serial.println(ALPHAangle[5][stepnum]);
  //Serial.println("*********ALPHA above********");
  //Serial.println(GAMMAangle[2][stepnum]);
  //Serial.println("*********GAMMA above********");
    // Serial.println("========================"); */
  }  
}


void assignToSSC32U (float BETA[][IccSteps], float ALPHA[][IccSteps], float GAMMA[][IccSteps] ){
  for (int stepNum=0; stepNum<IccSteps; stepNum++){
  SSC32.print("#20 P"); SSC32.print(BETA[LF][stepNum],DEC); SSC32.print("#1 P"); SSC32.print(ALPHA[LF][stepNum],DEC); SSC32.print("#2 P"); SSC32.print(GAMMA[LF][stepNum],DEC);   //SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
  SSC32.print("#3 P"); SSC32.print(BETA[LM][stepNum],DEC); SSC32.print("#4 P"); SSC32.print(ALPHA[LM][stepNum],DEC); SSC32.print("#5 P"); SSC32.print(GAMMA[LM][stepNum],DEC);   //SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
  SSC32.print("#6 P"); SSC32.print(BETA[LR][stepNum],DEC); SSC32.print("#7 P"); SSC32.print(ALPHA[LR][stepNum],DEC); SSC32.print("#8 P"); SSC32.print(GAMMA[LR][stepNum],DEC);   //SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
  SSC32.print("#9 P"); SSC32.print(BETA[RF][stepNum],DEC); SSC32.print("#10 P"); SSC32.print(ALPHA[RF][stepNum],DEC); SSC32.print("#11 P"); SSC32.print(GAMMA[RF][stepNum],DEC); //SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
  SSC32.print("#12 P"); SSC32.print(BETA[RM][stepNum],DEC); SSC32.print("#13 P"); SSC32.print(ALPHA[RM][stepNum],DEC); SSC32.print("#14 P"); SSC32.print(GAMMA[RM][stepNum],DEC);//SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
  SSC32.print("#15 P"); SSC32.print(BETA[RR][stepNum],DEC); SSC32.print("#16 P"); SSC32.print(ALPHA[RR][stepNum],DEC); SSC32.print("#17 P"); SSC32.print(GAMMA[RR][stepNum],DEC);
  SSC32.print("T"); SSC32.println(sscDelayTime,DEC);
  delay(50);
  }
}





   


  
//==========================================ICC for tripod gait (including rotation)====================================================

float IccRotationMatrix_X(float angles, float xPos, float yPos){
  float NewX=xPos*cos(angles)-yPos*sin(angles);
  return NewX;
}
float IccRotationMatrix_Y(float angles, float xPos, float yPos){
  float NewY=xPos*sin(angles)+yPos*cos(angles);
  return NewY;
}




void IccAlgorithms(float IccValue_X, float IccValue_Y, int value_for_sequenceStatus,float value_for_Z_Raise,float strideLength){

  for (int i=LF; i<=RR; i++){                                         //Calculating the distance from ICC to the foot points and the step angles for each stride 
    distanceIccToLeg[i]=sqrt(sq(IccValue_X-initial_X[i])+sq(IccValue_Y-initial_Y[i]));
    IccStepAngles[i]=(acos(1-(sq(strideLength)/(2*sq(distanceIccToLeg[i])))))/IccSteps;
  }
   //Serial.println(IccStepAngles[1],predigit);
 for (int i=LF;i<=RR; i++){                                          // Express iniital foot points in Icc coordinate frame.
  initialIccFrame_X[i]=initial_X[i]-IccValue_X;
  initialIccFrame_Y[i]=initial_Y[i]-IccValue_Y;
 }

  for (int stepNum=0; stepNum<IccSteps; stepNum++){                                                          //Create a 3*15 matrices for each foot point. 3 means x,y,z and 15 means 15 increments! and all 15 coordinates are expressed in the ICC coordinate.
    for (int legNum=LF; legNum<=RR; legNum++){
      if ( legNum == LF || legNum==LR || legNum==RM){
        FootPoint_X[legNum][stepNum]=IccRotationMatrix_X((value_for_sequenceStatus*(stepNum*IccStepAngles[legNum])),initialIccFrame_X[legNum],initialIccFrame_Y[legNum]);
        FootPoint_Y[legNum][stepNum]=IccRotationMatrix_Y((value_for_sequenceStatus*(stepNum*IccStepAngles[legNum])),initialIccFrame_X[legNum],initialIccFrame_Y[legNum]);
        if (      (0<=stepNum)  &&   (   stepNum<= ((IccSteps-1)/2.0)  )  ){
        FootPoint_Z[legNum][stepNum]=((Z_Height)+stepNum*(value_for_Z_Raise/((IccSteps-1)/2.0)));       
        }
        else if (((IccSteps-1)/2.0<stepNum)&&(stepNum<IccSteps)){
          FootPoint_Z[legNum][stepNum]=FootPoint_Z[legNum][(IccSteps-1)-stepNum];
        } 
      }
      else if(legNum == LM || legNum==RF ||  legNum==RR){
        FootPoint_X[legNum][stepNum]=IccRotationMatrix_X((-1*value_for_sequenceStatus*(stepNum*IccStepAngles[legNum])),initialIccFrame_X[legNum],initialIccFrame_Y[legNum]);
        FootPoint_Y[legNum][stepNum]=IccRotationMatrix_Y((-1*value_for_sequenceStatus*(stepNum*IccStepAngles[legNum])),initialIccFrame_X[legNum],initialIccFrame_Y[legNum]);
        FootPoint_Z[legNum][stepNum]=Z_Height;
      
      }
       
    }
    
  }

  for (int stepNum=0; stepNum<IccSteps; stepNum++){                                                         //Transform the footpoints_respect_to_ICC to footpoints_respect_to_bodycenter.
    for (int legNum=LF; legNum<=RR; legNum++){
      FootPoint_X[legNum][stepNum]=FootPoint_X[legNum][stepNum]+ICC[0].X;
      FootPoint_Y[legNum][stepNum]=FootPoint_Y[legNum][stepNum]+ICC[0].Y;
    }
  }


 coordinateTransformation(FootPoint_X , FootPoint_Y, FootPoint_Z);
 angleTransformation(FootPoint_X,  FootPoint_Y,  FootPoint_Z);
 assignToSSC32U(BETAangle,ALPHAangle,GAMMAangle);

for (int stepNum=0; stepNum<(IccSteps/2.0);stepNum++){
  for(int legNum=LF;legNum<=RR;legNum++){
    float temporary_X = FootPoint_X[legNum][stepNum];
    FootPoint_X[legNum][stepNum]=FootPoint_X[legNum][(IccSteps-1)-stepNum];
    FootPoint_X[legNum][(IccSteps-1)-stepNum]=temporary_X;
    float temporary_Y=FootPoint_Y[legNum][stepNum];
    FootPoint_Y[legNum][stepNum]=FootPoint_Y[legNum][(IccSteps-1)-stepNum];
    FootPoint_Y[legNum][(IccSteps-1)-stepNum]=temporary_Y;
  }
}
for (int stepNum=0; stepNum<IccSteps;stepNum++){
    for (int legNum=LF;legNum<=RR;legNum++){
    if( legNum == LM || legNum == RF || legNum == RR){
      if (  (0<=stepNum)&&(stepNum<=(IccSteps-1)/2.0) ) {
       FootPoint_Z[legNum][stepNum]=((Z_Height)+stepNum*(value_for_Z_Raise/((IccSteps-1)/2.0)));
      }
      else if (   (  ((IccSteps-1)/2.0)<stepNum  )  &&  (stepNum<IccSteps)  ){
        FootPoint_Z[legNum][stepNum]=FootPoint_Z[legNum][(IccSteps-1)-stepNum];
      }
    }
    else if (legNum == LF || legNum == LR || legNum == RM ){
      FootPoint_Z[legNum][stepNum]=Z_Height;
    }  
  } 
}

angleTransformation(FootPoint_X, FootPoint_Y, FootPoint_Z);
assignToSSC32U(BETAangle,ALPHAangle,GAMMAangle);
  
}

//***********************************************************RIPPLE GAIT****************************************************************************************
void coordinateTransformationRippleGait(float footPointPosition_X[][IccSteps * 2 - (IccSteps - 1) / 2], float footPointPosition_Y[][IccSteps * 2 - (IccSteps - 1) / 2], float footPointPosition_Z[][IccSteps * 2 - (IccSteps - 1) / 2]) {

  for (int stepnum = 0; stepnum<IccSteps * 2 - (IccSteps - 1) / 2; stepnum++) {
    for (int legnum = LF; legnum <= RR; legnum++) {

      footPointPosition_X[legnum][stepnum] = footPointPosition_X[legnum][stepnum] - CoxaToBodyPosition_X[legnum];
      footPointPosition_Y[legnum][stepnum] = footPointPosition_Y[legnum][stepnum] - CoxaToBodyPosition_Y[legnum];
      footPointPosition_Z[legnum][stepnum] = footPointPosition_Z[legnum][stepnum] - CoxaToBodyPosition_Z[legnum];

      float temporary_X = footPointPosition_X[legnum][stepnum];                                                                                             //                     -----------------
      footPointPosition_X[legnum][stepnum] = footPointPosition_X[legnum][stepnum] * cos(angleRotation[legnum]) - footPointPosition_Y[legnum][stepnum] * sin(angleRotation[legnum]);   //X'=X*cosT-Y*sinT              | Rotation      |
      footPointPosition_Y[legnum][stepnum] = (temporary_X)*sin(angleRotation[legnum]) + footPointPosition_Y[legnum][stepnum] * cos(angleRotation[legnum]);                          //Y'=X*sinT+Y*cosT              | matrix around |
      footPointPosition_Z[legnum][stepnum] = footPointPosition_Z[legnum][stepnum];                                                                                              //Z'=Z                          | Z axis        |
                                                                                            //                                                           ---------------                      
    }


  }

}
//IK algorithms to calculate new angles.
void rightShiftArray(int offset, int legNum, float footPointPosition_X[][IccSteps * 2 - (IccSteps - 1) / 2], float footPointPosition_Y[][IccSteps * 2 - (IccSteps - 1) / 2], float footPointPosition_Z[][IccSteps * 2 - (IccSteps - 1) / 2]) {
  for (int i = 0; i < offset; i++) {
    float temp1 = footPointPosition_X[legNum][IccSteps * 2 - ((IccSteps - 1) / 2) - 1];
    float temp2 = footPointPosition_Y[legNum][IccSteps * 2 - ((IccSteps - 1) / 2) - 1];
    float temp3 = footPointPosition_Z[legNum][IccSteps * 2 - ((IccSteps - 1) / 2) - 1];
    for (int j = IccSteps * 2 - ((IccSteps - 1) / 2) - 1; j > 0; j--) {
      footPointPosition_X[legNum][j] = footPointPosition_X[legNum][j - 1];
      footPointPosition_Y[legNum][j] = footPointPosition_Y[legNum][j - 1];
      footPointPosition_Z[legNum][j] = footPointPosition_Z[legNum][j - 1];
    }
    footPointPosition_X[legNum][0] = temp1;
    footPointPosition_Y[legNum][0] = temp2;
    footPointPosition_Z[legNum][0] = temp3;
  }
}
void angleTransformationRippleGait(float footPointPosition_X[][IccSteps * 2 - (IccSteps - 1) / 2], float footPointPosition_Y[][IccSteps * 2 - (IccSteps - 1) / 2], float footPointPosition_Z[][IccSteps * 2 - (IccSteps - 1) / 2]) {
  for (int stepnum = 0; stepnum<IccSteps * 2 - (IccSteps - 1) / 2; stepnum++) {
    for (int legnum = LF; legnum <= RR; legnum++) {
      float u = sq(sqrt(sq(footPointPosition_X[legnum][stepnum]) + sq(footPointPosition_Y[legnum][stepnum])) - virtualCoxa);
      BETAangleRippleGait[legnum][stepnum] = (acos((sq(Femur) + sq(Tibia) - u - sq(abs(footPointPosition_Z[legnum][stepnum]) - v)) / (2 * FT)))*radToDegree;
      ALPHAangleRippleGait[legnum][stepnum] = (acos((abs(footPointPosition_Z[legnum][stepnum]) - v) / (sqrt(u + sq(abs(footPointPosition_Z[legnum][stepnum]) - v)))) + acos((u + sq(abs(footPointPosition_Z[legnum][stepnum]) - v) + sq(Femur) - sq(Tibia)) / (2 * Femur*sqrt(u + sq(abs(footPointPosition_Z[legnum][stepnum]) - v)))))*radToDegree;
      GAMMAangleRippleGait[legnum][stepnum] = (atan2(footPointPosition_Y[legnum][stepnum], footPointPosition_X[legnum][stepnum]))*radToDegree;//-90
                                                                      //Serial.println(GAMMAangle[legnum][stepnum]);
                                                                      //Serial.println("===");
                                                                    //adding offset to angles
      if (legnum == 0) {
        BETAangleRippleGait[legnum][stepnum] = (180 - BETAangleRippleGait[legnum][stepnum]);
        //ALPHAangle stays the same
        GAMMAangleRippleGait[legnum][stepnum] = (180 - GAMMAangleRippleGait[legnum][stepnum] + 5);
      }
      else if (legnum == 1) {
        BETAangleRippleGait[legnum][stepnum] = (180 - BETAangleRippleGait[legnum][stepnum]);
        ALPHAangleRippleGait[legnum][stepnum] = (ALPHAangleRippleGait[legnum][stepnum] - 3);
        GAMMAangleRippleGait[legnum][stepnum] = (180 - GAMMAangleRippleGait[legnum][stepnum] + 3);
      }
      else if (legnum == 2) {
        BETAangleRippleGait[legnum][stepnum] = (180 - BETAangleRippleGait[legnum][stepnum] + 5);
        //ALPHAangle stays the same
        GAMMAangleRippleGait[legnum][stepnum] = (180 - GAMMAangleRippleGait[legnum][stepnum] + 3);
      }
      else if (legnum == 3) {
        //BETAangle stays the same
        ALPHAangleRippleGait[legnum][stepnum] = (180 - ALPHAangleRippleGait[legnum][stepnum] - 3);
        GAMMAangleRippleGait[legnum][stepnum] = (180 - GAMMAangleRippleGait[legnum][stepnum] + 3);
      }
      else if (legnum == 4) {
        //BETAangle stays the same
        ALPHAangleRippleGait[legnum][stepnum] = (180 - ALPHAangleRippleGait[legnum][stepnum] + 7);
        GAMMAangleRippleGait[legnum][stepnum] = (180 - GAMMAangleRippleGait[legnum][stepnum] + 5);
      }
      else if (legnum == 5) {
        //BETAangle stays the same
        ALPHAangleRippleGait[legnum][stepnum] = (180 - ALPHAangleRippleGait[legnum][stepnum] - 5);
        GAMMAangleRippleGait[legnum][stepnum] = (180 - GAMMAangleRippleGait[legnum][stepnum] + 5);
      }

      
      BETAangleRippleGait[legnum][stepnum] = (BETAangleRippleGait[legnum][stepnum] - 30.0)*((2166.0 - 834.0) / (150.0 - 30.0)) + 834.0;//                   map(BETAangle[legnum][stepnum],30.0,150.0,834.0,2166.0);
      ALPHAangleRippleGait[legnum][stepnum] = (ALPHAangleRippleGait[legnum][stepnum] - 30.0)*((2166.0 - 834.0) / (150.0 - 30.0)) + 834.0;  //                                      map(ALPHAangle[legnum][stepnum],30.0,150.0,834.0,2166.0);
      GAMMAangleRippleGait[legnum][stepnum] = (GAMMAangleRippleGait[legnum][stepnum] - 30.0)*((2166.0 - 834.0) / (150.0 - 30.0)) + 834.0;//                   map(GAMMAangle[legnum][stepnum],30.0,150.0,834.0,2166.0);
    }
  }
}
void assignToSSC32URippleGait(float BETA[][IccSteps * 2 - (IccSteps - 1) / 2], float ALPHA[][IccSteps * 2 - (IccSteps - 1) / 2], float GAMMA[][IccSteps * 2 - (IccSteps - 1) / 2]) {
  for (int stepNum = 0; stepNum<IccSteps * 2 - (IccSteps - 1) / 2; stepNum++) {
    SSC32.print("#20 P"); SSC32.print(BETA[LF][stepNum], DEC); SSC32.print("#1 P"); SSC32.print(ALPHA[LF][stepNum], DEC); SSC32.print("#2 P"); SSC32.print(GAMMA[LF][stepNum], DEC);   //SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
    SSC32.print("#3 P"); SSC32.print(BETA[LM][stepNum], DEC); SSC32.print("#4 P"); SSC32.print(ALPHA[LM][stepNum], DEC); SSC32.print("#5 P"); SSC32.print(GAMMA[LM][stepNum], DEC);   //SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
    SSC32.print("#6 P"); SSC32.print(BETA[LR][stepNum], DEC); SSC32.print("#7 P"); SSC32.print(ALPHA[LR][stepNum], DEC); SSC32.print("#8 P"); SSC32.print(GAMMA[LR][stepNum], DEC);   //SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
    SSC32.print("#9 P"); SSC32.print(BETA[RF][stepNum], DEC); SSC32.print("#10 P"); SSC32.print(ALPHA[RF][stepNum], DEC); SSC32.print("#11 P"); SSC32.print(GAMMA[RF][stepNum], DEC); //SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
    SSC32.print("#12 P"); SSC32.print(BETA[RM][stepNum], DEC); SSC32.print("#13 P"); SSC32.print(ALPHA[RM][stepNum], DEC); SSC32.print("#14 P"); SSC32.print(GAMMA[RM][stepNum], DEC);//SSC32.print("T"); SSC32.println(sscDelayTime,DEC);delay(sscDelayTime);
    SSC32.print("#15 P"); SSC32.print(BETA[RR][stepNum], DEC); SSC32.print("#16 P"); SSC32.print(ALPHA[RR][stepNum], DEC); SSC32.print("#17 P"); SSC32.print(GAMMA[RR][stepNum], DEC);
    SSC32.print("T"); SSC32.println(300, DEC);
    delay(50);
  }
}
void IccAlgorithmsRippleGait(float IccValue_X, float IccValue_Y, int value_for_sequenceStatus, float value_for_Z_Raise,float strideLength) {

  for (int i = LF; i <= RR; i++) {                                         //Calculating the distance from ICC to the foot points and the step angles for each stride 
    distanceIccToLeg[i] = sqrt(sq(IccValue_X - initial_X[i]) + sq(IccValue_Y - initial_Y[i]));
    IccStepAngles[i] = (acos(1 - (sq(strideLength) / (2 * sq(distanceIccToLeg[i]))))) / IccSteps; 
  }
  //Serial.println(IccStepAngles[1],predigit);
  for (int i = LF; i <= RR; i++) {                                          // Express iniital foot points in Icc coordinate frame.
    initialIccFrame_X[i] = initial_X[i] - IccValue_X;
    initialIccFrame_Y[i] = initial_Y[i] - IccValue_Y;
  }

  for (int stepNum = 0; stepNum<IccSteps; stepNum++) {                                                          //Create a 3*15 matrices for each foot point. 3 means x,y,z and 15 means 15 increments! and all 15 coordinates are expressed in the ICC coordinate.
    for (int legNum = LF; legNum <= RR; legNum++) {
      if (legNum == LF || legNum == LR || legNum == RM) {
        FootPoint_X[legNum][stepNum] = IccRotationMatrix_X((value_for_sequenceStatus*(stepNum*IccStepAngles[legNum])), initialIccFrame_X[legNum], initialIccFrame_Y[legNum]);
        FootPoint_Y[legNum][stepNum] = IccRotationMatrix_Y((value_for_sequenceStatus*(stepNum*IccStepAngles[legNum])), initialIccFrame_X[legNum], initialIccFrame_Y[legNum]);
        if ((0 <= stepNum) && (stepNum <= ((IccSteps - 1) / 2.0))) {
          FootPoint_Z[legNum][stepNum] = ((Z_Height)+stepNum * (value_for_Z_Raise / ((IccSteps - 1) / 2.0)));
        }
        else if (((IccSteps - 1) / 2.0<stepNum) && (stepNum<IccSteps)) {
          FootPoint_Z[legNum][stepNum] = FootPoint_Z[legNum][(IccSteps - 1) - stepNum];
        }
      }
      else if (legNum == LM || legNum == RF || legNum == RR) {
        FootPoint_X[legNum][stepNum] = IccRotationMatrix_X((-1 * value_for_sequenceStatus*(stepNum*IccStepAngles[legNum])), initialIccFrame_X[legNum], initialIccFrame_Y[legNum]);
        FootPoint_Y[legNum][stepNum] = IccRotationMatrix_Y((-1 * value_for_sequenceStatus*(stepNum*IccStepAngles[legNum])), initialIccFrame_X[legNum], initialIccFrame_Y[legNum]);
        FootPoint_Z[legNum][stepNum] = Z_Height;

      }

    }

  }
// initialize temporary double size matrix for ripple gait.
  float TempFootPoint_X[6][IccSteps * 2 - (IccSteps - 1) / 2] = {};
  float TempFootPoint_Y[6][IccSteps * 2 - (IccSteps - 1) / 2] = {};
  float TempFootPoint_Z[6][IccSteps * 2 - (IccSteps - 1) / 2] = {};

  for (int stepNum = 0; stepNum<IccSteps; stepNum++) {                                                         //Transform the footpoints_respect_to_ICC to footpoints_respect_to_bodycenter.
    for (int legNum = LF; legNum <= RR; legNum++) {
      FootPoint_X[legNum][stepNum] = FootPoint_X[legNum][stepNum] + ICC[0].X;
      FootPoint_Y[legNum][stepNum] = FootPoint_Y[legNum][stepNum] + ICC[0].Y;
    }
  }

  
  for (int stepNum = 0; stepNum<IccSteps; stepNum++) {                                                         
    for (int legNum = LF; legNum <= RR; legNum++) {
       if (legNum == LF || legNum == LR || legNum == RM) {
        if (stepNum <= (IccSteps / 2)) {
          TempFootPoint_X[legNum][stepNum] = FootPoint_X[legNum][stepNum*2];           // take off foot points for swing phase leg.
          TempFootPoint_Y[legNum][stepNum] = FootPoint_Y[legNum][stepNum*2];
          TempFootPoint_Z[legNum][stepNum] = FootPoint_Z[legNum][stepNum*2];
        }
      }
      else if (legNum == LM || legNum == RF || legNum == RR) {
        TempFootPoint_X[legNum][stepNum] = FootPoint_X[legNum][stepNum];
        TempFootPoint_Y[legNum][stepNum] = FootPoint_Y[legNum][stepNum];
        TempFootPoint_Z[legNum][stepNum] = FootPoint_Z[legNum][stepNum];
      }
    }
  
  }


  for (int stepNum = 0; stepNum<(IccSteps / 2.0); stepNum++) {
    for (int legNum = LF; legNum <= RR; legNum++) {
      float temporary_X = FootPoint_X[legNum][stepNum];
      FootPoint_X[legNum][stepNum] = FootPoint_X[legNum][(IccSteps - 1) - stepNum];
      FootPoint_X[legNum][(IccSteps - 1) - stepNum] = temporary_X;
      float temporary_Y = FootPoint_Y[legNum][stepNum];
      FootPoint_Y[legNum][stepNum] = FootPoint_Y[legNum][(IccSteps - 1) - stepNum];
      FootPoint_Y[legNum][(IccSteps - 1) - stepNum] = temporary_Y;
    }
  }
  for (int stepNum = 0; stepNum<IccSteps; stepNum++) {
    for (int legNum = LF; legNum <= RR; legNum++) {
      if (legNum == LM || legNum == RF || legNum == RR) {
        if ((0 <= stepNum) && (stepNum <= (IccSteps - 1) / 2.0)) {
          FootPoint_Z[legNum][stepNum] = ((Z_Height)+stepNum * (value_for_Z_Raise / ((IccSteps - 1) / 2.0)));
        }
        else if ((((IccSteps - 1) / 2.0)<stepNum) && (stepNum<IccSteps)) {
          FootPoint_Z[legNum][stepNum] = FootPoint_Z[legNum][(IccSteps - 1) - stepNum];
        }
      }
      else if (legNum == LF || legNum == LR || legNum == RM) {
        FootPoint_Z[legNum][stepNum] = Z_Height;
      }
    }
  }

  for (int stepNum = 0; stepNum < IccSteps; stepNum++) {
    for (int legNum = LF; legNum <= RR; legNum++) {
      if (legNum == LM || legNum == RF || legNum == RR) {
        if (stepNum <= (IccSteps / 2)) {
          TempFootPoint_X[legNum][stepNum + IccSteps] = FootPoint_X[legNum][stepNum * 2];           // take off foot points for swing phase leg.
          TempFootPoint_Y[legNum][stepNum + IccSteps] = FootPoint_Y[legNum][stepNum * 2];
          TempFootPoint_Z[legNum][stepNum + IccSteps] = FootPoint_Z[legNum][stepNum * 2];
        }
      }
      else if (legNum == LF || legNum == LR || legNum == RM) {
        TempFootPoint_X[legNum][1 + stepNum + IccSteps / 2] = FootPoint_X[legNum][stepNum];
        TempFootPoint_Y[legNum][1 + stepNum + IccSteps / 2] = FootPoint_Y[legNum][stepNum];
        TempFootPoint_Z[legNum][1 + stepNum + IccSteps / 2] = FootPoint_Z[legNum][stepNum];
      }
    }
  }
  int rippleLegOffset[6] = {0,0,4,12,2,2};
  
  //shift the position array for ripple gait.
  for (int i = 0; i < 6; i++) {
    rightShiftArray(rippleLegOffset[i], i, TempFootPoint_X, TempFootPoint_Y, TempFootPoint_Z);
  }




  coordinateTransformationRippleGait(TempFootPoint_X, TempFootPoint_Y, TempFootPoint_Z);
  angleTransformationRippleGait(TempFootPoint_X, TempFootPoint_Y, TempFootPoint_Z);
  assignToSSC32URippleGait(BETAangleRippleGait, ALPHAangleRippleGait, GAMMAangleRippleGait);
}


//======================================================PS2 to ICC==============================

float JoystickToICC_X(int X, int Y,int distance){
  return distance*cos((atan2((Y-49),(X-49))+pi/2.0));
}
float JoystickToICC_Y(int X, int Y,int distance){
  return distance*sin(atan2((Y-49),(X-49))+pi/2.0);
}






























  
