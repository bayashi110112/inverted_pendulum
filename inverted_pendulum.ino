#include <MsTimer2.h>

float pot_target = 408.5;
float pot_current = 0;
float old_err = 0;  //前回の偏差
float err = 0;  //偏差
float err_Integral = 0; //偏差の積分
float output, pid, P, I, D;

/////// パラメータ ///////
const double Kp = 15.0; 
const double Ki = 0.03;  
const double Kd = 8.0; 

void setup() {
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  MsTimer2::set(1, pid_ctl); // 割り込み関数定義 1msごとにオンオフ
  MsTimer2::start();
}

void loop() {
  if (output >= 0) {
    analogWrite(6, output);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
  } else {
    analogWrite(6, -output);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
  }
}

//PID制御
void pid_ctl() {
  pot_current = analogRead(A5);
  old_err = err;
  err = pot_target - pot_current;       //偏差=目標値-制御値
  err_Integral += err * 0.001;   //偏差の積分 1ms

  P = Kp * err;
  I = Ki * err_Integral;
  D = Kd * (err - old_err) / 0.001;
  pid = P + I + D;

  output = constrain(pid, -255, 255); //-255~255範囲
}
