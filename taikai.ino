#include <ros.h>



#include <geometry_msgs/Twist.h>



#include <FlexiTimer2.h>



#include <Wire.h>







ros::NodeHandle nh;







//非常停止



int W_flag = 0;



static int hako = 0;



static byte data[8] = {



};







//エンコーダ



#define ENC_HOWMANY 3



#define Bunkainou 2048



int moter_A[ENC_HOWMANY]  = {



  2, 3, 18

};



int moter_B[ENC_HOWMANY]  = {



  42, 44, 46

};



volatile char ENC_old_Rot[ENC_HOWMANY] = {



};



volatile double ENC_Rot[ENC_HOWMANY]  = {



};







//moterのpin(Dual)



#define Moter_HOWMANY 5



short TSpeed[Moter_HOWMANY] = {



  0

}



, Speed[Moter_HOWMANY] = {



  0

};



const byte pinPWM[Moter_HOWMANY] =     {



  6, 7, 8, 11, 12

};



const byte pinDualHLA[Moter_HOWMANY] = {



  22, 24, 26, 28, 30

};



const byte pinDualHLB[Moter_HOWMANY] = {



  32, 34, 36, 38, 40

};







//速さの計算



//#define PI 3.14159265



#define DIAmeter 0.11//タイヤの直径[m]



#define Circumference DIAmeter * PI//タイヤの円周[m]



#define Radius 0.46 //ロボットの半径[m]



double Linear_x = 0;//Xの速さ



double Linear_y = 0;//Yの速さ



double angle_z = 0; //Zの角速度







//青基盤



byte mekanaru[11] = {



  0x01, 0, 0, 0, 0, 0, 0, 0x00, 0x00, 0x0d, 0x0a

};



int BL_Vx, BL_Vy, BL_Vz;



double cmd_vx, cmd_vy, cmd_vth;



int V, deg;







//超音波の変数



unsigned long PW = 0;



int Ultrasonic[3] = {



  A1, A2, A3

};



int Length[2] = {



  0, 0

};





#define AV 5

int i = 0;



int VX1[5] = {

};

int VX2[5] = {

};

int VY1[5] = {

};





int j = 0;



void velCallback(const geometry_msgs::Twist& vel) {



  cmd_vx = vel.linear.x;



  cmd_vy = vel.linear.y;



  cmd_vth = vel.angular.z;



}







ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);



geometry_msgs::Twist twist;



ros::Publisher state("state", &twist);







void setup() {



  nh.initNode();



  nh.subscribe(sub);



  nh.advertise(state);







  Serial3.begin(57600);



  Serial.begin(57600);

  Serial2.begin(57600);



  Wire.begin(0x08);



  Wire.onReceive(receiveEvent);



  for (i = 0; i < Moter_HOWMANY; i++)



    pinMode(pinPWM[i], OUTPUT);



  for (i = 0; i < Moter_HOWMANY; i++)



    pinMode(pinDualHLA[i], OUTPUT);



  for (i = 0; i < Moter_HOWMANY; i++)



    pinMode(pinDualHLB[i], OUTPUT);



  TCCR1A = B10100001;



  TCCR1B = B00001010;



  TCCR4A = B10100001;



  TCCR4B = B00001010;







  for (i = 0; i < ENC_HOWMANY; i++)



    pinMode(moter_A[i], INPUT);



  for (i = 0; i < ENC_HOWMANY; i++)



    pinMode(moter_B[i], INPUT);







  attachInterrupt(0, ROT_VX_1, CHANGE);



  attachInterrupt(1, ROT_VY_1, CHANGE);



  attachInterrupt(5, ROT_VX_2, CHANGE);







  FlexiTimer2::set(10, ROSserial); // MsTimer2 style is also supported



  FlexiTimer2::start();



}











void ROSserial() {



  double Vx1 = (ENC_Rot[0] * 100.0) * Circumference / Bunkainou;



  double Vy1 = (ENC_Rot[1] * 100.0 ) * Circumference / Bunkainou;



  double Vx2 = (ENC_Rot[2] * 100.0 ) * Circumference / Bunkainou;







  //  VX1[j] = Vx1;

  //

  //  VY1[j] = Vy1;

  //

  //  VX2[j] = Vx2;

  //

  //  j++;

  //

  //  if(j == AV)

  //

  //    j = 0;

  //

  //  for(j = 0;j < AV; j++){

  //

  //    Vx1 += VX1[j];

  //

  //    Vy1 += VY1[j];

  //

  //    Vx2 += VX2[j];

  //

  //  }

  //

  //  Vx1 = Vx1 / AV;

  //

  //  Vy1 = Vy1 / AV;

  //

  //  Vx2 = Vx2 / AV;





  Linear_x = ((Vx1 + Vx2) / 2);



  Linear_y = Vy1;



  angle_z = (((ENC_Rot[0] - ENC_Rot[2]) * 100) * Circumference / Bunkainou / (Radius)) * -1;

  //angle_z = 1;





  //  Serial2.print(Linear_x);



  //  Serial2.print("\t");



  //  Serial2.print(Linear_y);



  //  Serial2.print("\t");



  //  Serial2.println(angle_z);



  ENC_Rot[0] = 0;



  ENC_Rot[1] = 0;



  ENC_Rot[2] = 0;



}















void loop() {



  twist.linear.x = Linear_x;



  twist.linear.y = Linear_y;



  twist.angular.z = angle_z;







  state.publish(&twist);







  nh.spinOnce();







  //  Serial.print(Linear_x);



  //  Serial.print("\t");



  //  Serial.print(Linear_y);



  //  Serial.print("\t");



  //  Serial.print(angle_z);



  //  Serial.print("\t");



  //  Serial.print(ENC_Rot[0]);



  //  Serial.print("\t");



  //  Serial.print(ENC_Rot[1]);



  //  Serial.print("\t");



  //  Serial.println(ENC_Rot[2]);







  if (W_flag == 0) {



    if (data[1] == 1)

      W_flag = 1;



    BL_Circuit_mosion();



    TSpeed[0] = 100;



    TSpeed[1] = 100;



    TSpeed[2] = 100;



    TSpeed[3] = 100;



    TSpeed[4] = 100;



  }



  else if (W_flag == 1) {



    BL_Circuit_END();



    TSpeed[0] = 0;



    TSpeed[1] = 0;



    TSpeed[2] = 0;



    TSpeed[3] = 0;



    TSpeed[4] = 0;



  }



  PWM_Acc();



}



void PWM_Acc() {







  for (i = 0; i < Moter_HOWMANY; i++) {



    if (Speed[i] < TSpeed[i])



      Speed[i]++;



    else if (Speed[i] > TSpeed[i])



      Speed[i]--;







    if (i < Moter_HOWMANY)



      DC_Dual(i, Speed[i]);



  }







}











void DC_Dual(byte MotorNum, short Val) {







  byte DualHLAflg = LOW, DualHLBflg = LOW;







  if (Val > 0) {



    DualHLAflg = LOW;



    DualHLBflg = HIGH;



  }



  else if (Val < 0) {



    DualHLAflg = HIGH;



    DualHLBflg = LOW;



    Val *= -1;



  }







  digitalWrite(pinDualHLA[MotorNum], DualHLAflg);



  digitalWrite(pinDualHLB[MotorNum], DualHLBflg);



  analogWrite(pinPWM[MotorNum], Val);







}







void ROT_VX_1(void) {



  int  i = 0;



  if (!digitalRead(moter_A[i])) { // ロータリーエンコーダー回転開始



    if (digitalRead(moter_B[i]))



      ENC_old_Rot[i] = 'R';    //右回転



    else



        ENC_old_Rot[i] = 'L';    //左回転



  }



  else {  // ロータリーエンコーダー回転停止



    if (digitalRead(moter_B[i])) {



      if (ENC_old_Rot[i] == 'L')  // 左回転の時の処理



        ENC_Rot[i]++;



    }



    else {



      if (ENC_old_Rot[i] == 'R')   //右回転の時の処理



        ENC_Rot[i]--;



    }



    ENC_old_Rot[i] = 0;    // ここでロータリーエンコーダーの状態をクリア



  }



}











void ROT_VY_1(void) {



  int  i = 1;



  if (!digitalRead(moter_A[i])) { // ロータリーエンコーダー回転開始



    if (digitalRead(moter_B[i]))



      ENC_old_Rot[i] = 'R';    //右回転



    else



        ENC_old_Rot[i] = 'L';    //左回転



  }



  else {  // ロータリーエンコーダー回転停止



    if (digitalRead(moter_B[i])) {



      if (ENC_old_Rot[i] == 'L')  // 左回転の時の処理



        ENC_Rot[i]++;



    }



    else {



      if (ENC_old_Rot[i] == 'R')   //右回転の時の処理



        ENC_Rot[i]--;



    }



    ENC_old_Rot[i] = 0;    // ここでロータリーエンコーダーの状態をクリア



  }



}











void ROT_VX_2(void) {



  int  i = 2;



  if (!digitalRead(moter_A[i])) { // ロータリーエンコーダー回転開始



    if (digitalRead(moter_B[i]))



      ENC_old_Rot[i] = 'R';    //右回転



    else



        ENC_old_Rot[i] = 'L';    //左回転



  }



  else {  // ロータリーエンコーダー回転停止



    if (digitalRead(moter_B[i])) {



      if (ENC_old_Rot[i] == 'L')  // 左回転の時の処理



        ENC_Rot[i]--;



    }



    else {



      if (ENC_old_Rot[i] == 'R')   //右回転の時の処理



        ENC_Rot[i]++;



    }



    ENC_old_Rot[i] = 0;    // ここでロータリーエンコーダーの状態をクリア



  }



}





/*

 void Ultra_sonic() {//超音波の関数
 
 
 
 for (int i = 0; i < 3; i++) {
 
 
 
 pinMode(Ultrasonic[i], OUTPUT);
 
 
 
 digitalWrite(Ultrasonic[i], LOW);
 
 
 
 delayMicroseconds(2);
 
 
 
 digitalWrite(Ultrasonic[i], HIGH);
 
 
 
 delayMicroseconds(5);
 
 
 
 digitalWrite(Ultrasonic[i], LOW);
 
 
 
 delayMicroseconds(350);
 
 
 
 pinMode(Ultrasonic[i], INPUT);
 
 
 
 PW = pulseIn(Ultrasonic[i], HIGH);
 
 
 
 Length[i] = (float)PW * 0.34442 / 2.0;
 
 
 
 delay(10);
 
 
 
 }
 
 
 
 }*/







void BL_Circuit_mosion() {

  BL_Vx = cmd_vy * 250;



  BL_Vy = cmd_vx * 250;



  BL_Vz = cmd_vth * 400;



  //  BL_Vx = 100;

  //

  //  BL_Vy = 100;

  //

  //  BL_Vz = 100;



  V = sqrt(pow(abs(BL_Vx), 2) + pow(abs(BL_Vy), 2));



  if (V > 0x7F)



    V = 0x7F;



  deg = atan2(BL_Vy, BL_Vx) * (180 / PI);



  if (BL_Vy < 0)



    deg = deg + 360.0;



  deg = deg / 360.0 * 0xFF;



  data[1] = V;

  data[2] = deg;

  //0x7F : data[1] = 0x7FFF : mekanaru[1]のLOW byte



  mekanaru[1] = (data[1] * (0x7FFF / 0x7F)) >> 8;



  //0x7F : data[1] = 0x7FFF : mekanaru[2]のHIGH byte



  mekanaru[2] = (data[1] * (0x7FFF / 0x7F)) & 0xFF;

  //0xFF : data[2] = 0xFFFF : mekanaru[3]のHIGH byte



  mekanaru[3] = (data[2] * (0xFFFF / 0xFF)) >> 8;



  //0xFF : data[2] = 0xFFFF : mekanaru[4]のLOW byte



  mekanaru[4] = (data[2] * (0xFFFF / 0xFF)) & 0xFF;



  //回転



  mekanaru[5] = (BL_Vz) >> 8;



  mekanaru[6] = BL_Vz & 0xFF;



  for (int i = 0; i < 11; i++) {



    Serial3.write(mekanaru[i]);



    //    Serial1.print(mekanaru[i], HEX);



    //    Serial1.print(":");



  }



  // Serial1.println("");



}











void BL_Circuit_END() {



  BL_Vx = 0;



  BL_Vy = 0;



  BL_Vz = 0;



  //速度



  mekanaru[1] = BL_Vx >> 8;



  mekanaru[2] = BL_Vx & 0xFF;



  //角度



  mekanaru[3] = BL_Vy >> 8;



  mekanaru[4] = BL_Vy & 0xFF;



  //回転



  mekanaru[5] = (BL_Vz) >> 8;



  mekanaru[6] = BL_Vz & 0xFF;



  for (int i = 0; i < 11; i++) {



    Serial3.write(mekanaru[i]);

    //    Serial2.write(mekanaru[i]);



    //    Serial1.print(mekanaru[i], HEX);



    //    Serial1.print(":");



  }



  // Serial1.println("");



}







void receiveEvent(int howmany)



{



  while (Wire.available() == 0);



  byte I2C_RCVBuff = Wire.read();







  data[hako] = I2C_RCVBuff;







  if (data[hako] == 0x80)



    hako = 0;







  if (hako++ == 7) {



    hako = 0;



  }







  for (; howmany > 1; howmany--)



    I2C_RCVBuff = Wire.read();



}

