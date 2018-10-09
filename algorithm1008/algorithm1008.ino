/**




*/
#include <Stepper.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#define STEPS 200  //定義步進馬達每圈的步數
SoftwareSerial Bluetooth(12, 13); //藍芽
Stepper Lstepper(STEPS, 4, 5, 2, 3);
Stepper Rstepper(STEPS, 8, 9, 6, 7);
double a[2]={0};
double *v=a;   
const int Speed = 60;
const int enlarge = 10000;
const float  RobotPace = 1;	//待訂
const int goalC = 1000;	//待訂
const float R0[4] = {0.28, 0.36, 0.33, 0.28}; //0.1.3.4 R0 要校準
const int delaytime = 30;	//待訂
const int mq3[4] = {A0, A1, A2, A3};//gas sensors
double sensorvalues[3][4]={0};  //four sensors value for three points
double X[3][3] = {0}; //[x y concentration]x3
double Xref[3] = {0};
double Y[12][3] = {0};
byte car_direction = 0;//1:right  2:forward 3:left  4:back
void setup() {
  Serial.begin (9600);
  pinMode(10, OUTPUT);//Trig Pin
  pinMode(11, INPUT);//Echo Pin
  Bluetooth.begin(9600);
  Lstepper.setSpeed(Speed);
  Rstepper.setSpeed(Speed);
  /**
     X1
  */{
    X[0][0] = 0;
    X[0][1] = 0;
    for (int i = 0; i < 4; i++) {
      sensorvalues[0][i] = getppm(i);
      X[0][2] += 0.25 * sensorvalues[0][i];
    }

    Serial.print("C(X1)=");
    Serial.println(X[0][2]);
    forward();
  
}
/**
   X2
*/{
  X[1][0] = 0;
  X[1][1] = 1;
  for (int i = 0; i < 4; i++) {
    sensorvalues[1][i] = getppm(i);
  }
  getC(sensorvalues, 1);
  Serial.print("C(X2)=");
  Serial.println(X[1][2]);
}
/**
   X3
*/
{
  inputY(X[1][0] + 1, X[1][1], 0, 0);
  inputY(X[1][0] - 1, X[1][1], 1, 0);
  inputY(X[1][0], X[1][1] + 1, 2, 0);
  Serial.println("-----------------------------------------------");
  for (int i = 0; i < 2; i++) {
    if (Y[i][2] > Y[i + 1][2]) {
      X[2][0] = Y[i][0];
      X[2][1] = Y[i][1];
    } else {
      X[2][0] = Y[i + 1][0];
      X[2][1] = Y[i + 1][1];
    }
  }
  v = vector(X[1][0], X[1][1], X[2][0], X[2][1]);
 /* Serial.print("vector=");
  Serial.print(*v);
  Serial.print(",");
  Serial.println(*(v + 1));*/
  if (*(v + 1) > 0) {
    forward();
    car_direction = 2;
  } else if (*v > 0) {
    right();
    car_direction = 1;
  } else {
    left();
    car_direction = 3;
  }
  sensorRead();
}
}
void loop() {
  while (X[0][2] < goalC && X[1][2] < goalC && X[2][2] && goalC) {
    //predict around points
    { 
      Serial.println("-------------------------------------------P1--");
      inputY(X[0][0] + 1, X[0][1], 0, 1);
      inputY(X[0][0] - 1, X[0][1], 1, 1);
      inputY(X[1][0] + 1, X[1][1], 2, 1);
      inputY(X[1][0] - 1, X[1][1], 3, 1);
      inputY(X[2][0] + 1, X[2][1], 4, 1);
      inputY(X[2][0] - 1, X[2][1], 5, 1);
      inputY(X[0][0], X[0][1] + 1, 6 , 1);
      inputY(X[0][0], X[0][1] - 1, 7, 1);
      inputY(X[1][0], X[1][1] + 1, 8, 1);
      inputY(X[1][0], X[1][1] - 1, 9 , 1);
      inputY(X[2][0], X[2][1] + 1, 10 , 1);
      inputY(X[2][0], X[2][1] - 1, 11, 1);
      /*
        for (int i = 0; i < 3; i++) {
        inputY(X[i][0] + 1, X[i][1], i, 1);
        inputY(X[i][0] - 1, X[i][1], i + 1, 1);
        }
        for (int i = 0; i < 3; i++) {
        inputY(X[i][0], X[i][1] + 1, i + 7, 1);
        inputY(X[i][0], X[i][1] - 1, i + 8, 1);
        }*/
    }
    int count = 0; //count P1(Y)<0
    //find Xref
    {
      for (int j = 0; j < 11; j++) {
        if (Y[j][2] > 0) {
          if (Y[j][2] > Y[j + 1][2]) {
            Xref[0] = Y[j][0];
            Xref[1] = Y[j][1];
          } else {
            Xref[0] = Y[j + 1][0];
            Xref[1] = Y[j + 1][1];
          }
        } else {      
          count++;
          //continue;
        }
      }
    }

    if (count < 11) {//P1 is available
      v = vector(X[2][0], X[2][1], Xref[0], Xref[1]);
      Serial.print("vectorCheck=");
      Serial.print(*v);
      Serial.print(",");
      Serial.println(*(v + 1));
      //if is selected from W1(X3)
      if (distance(Xref[0], Xref[1], X[2][0], X[2][1]) == 1 / RobotPace) {
        switch (car_direction) {
          case 1:
            if (*v > 0 ) {
              forward();
              car_direction = 1;
            }
            else if (*(v + 1) >= 0) {
              left();
              car_direction = 2;
            }

            else {
              right();
              car_direction = 4;
            }
            break;
          case 2:
            if (*(v + 1) > 0) {
              forward();
              car_direction = 2;
            }

            else if (*v > 0 ) {
              right();
              car_direction = 1;
            }

            else {
              left();
              car_direction = 3;
            }

            break;
          case 3:
            if (*v < 0 ) {
              forward();
              car_direction = 3;
            }

            else if (*(v + 1) > 0) {
              right();
              car_direction = 2;
            }

            else {
              left();
              car_direction = 4;
            }

            break;
          case 4:
            if (*(v + 1) < 0) {
              forward();
              car_direction = 4;
            }

            else if (*v > 0) {
              left();
              car_direction = 1;
            }

            else {
              right();
              car_direction = 3;
            }

            break;
        }
        //換座標,濃度
        change(X[0], X[1], X[2], Xref, sensorvalues);
        //sensor read
        sensorRead();
        //if is selected from W1(X2) or W1(X1)
      } else if (distance(Xref[0], Xref[1], X[2][0], X[2][1]) > 1 / RobotPace) {
        switch (car_direction) {
          case 1:
            if (*(v + 1) >= 0) {
              left();
              car_direction = 2;
              Xref[0] = X[2][0];
              Xref[1] = X[2][1] + 1;
            }

            
            else {
              right();
              car_direction = 4;
              Xref[0] = X[2][0];
              Xref[1] = X[2][1] - 1;
            }

            break;
          case 2:
            if (*(v) >= 0) {
              right();
              car_direction = 1;
              Xref[0] = X[2][0] + 1;
              Xref[1] = X[2][1];
            }

            else {
              left();
              car_direction = 3;
              Xref[0] = X[2][0] - 1;
              Xref[1] = X[2][1];
            }

            break;
          case 3:
            if (*(v + 1) >= 0) {
              right();
              car_direction = 2;
              Xref[0] = X[2][0];
              Xref[1] = X[2][1] + 1;
            }

            else {
              left();
              car_direction = 4;
              Xref[0] = X[2][0];
              Xref[1] = X[2][1] - 1;
            }

            break;
          case 4:
            if (*(v) >= 0) {
              left();
              car_direction = 1;
              Xref[0] = X[2][0] + 1;
              Xref[1] = X[2][1];
            }

            else {
              right();
              car_direction = 3;
              Xref[0] = X[2][0] - 1;
              Xref[1] = X[2][1];
            }

            break;
        }
        change(X[0], X[1], X[2], Xref, sensorvalues);
        sensorRead();
      }
      //if no point available in W1(X3), use P2 to predict
    } else { //P2
      /*
        inputY(X[2][0] + 1, X[2][1], 0, 2);
        inputY(X[2][0] - 1, X[2][1], 1, 2);
        inputY(X[2][0] , X[2][1] + 1, 2, 2);
        inputY(X[2][0] , X[2][1] - 1, 3, 2);*/

      switch (car_direction) {
        case 1:
          inputY(X[2][0] + 1, X[2][1], 0, 2);
          inputY(X[2][0] , X[2][1] + 1, 1, 2);
          inputY(X[2][0] , X[2][1] - 1, 2, 2);
          for (int j = 0; j < 2; j++) {
            if (Y[j][2] > Y[j + 1][2]) {
              Xref[0] = Y[j][0];
              Xref[1] = Y[j][1];
            } else {
              Xref[0] = Y[j + 1][0];
              Xref[1] = Y[j + 1][1];
            }
          }
          v = vector(X[2][0], X[2][1], Xref[0], Xref[1]);
          if (*(v) > 0) {
            forward();
            car_direction = 1;
          }

          else if (*(v + 1) >= 0) {
            left();
            car_direction = 2;
          }

          else {
            right();
            car_direction = 4;
          }

          break;
        case 2:
          inputY(X[2][0] + 1, X[2][1], 0, 2);
          inputY(X[2][0] - 1, X[2][1], 1, 2);
          inputY(X[2][0] , X[2][1] + 1, 2, 2);
          for (int j = 0; j < 2; j++) {
            if (Y[j][2] > Y[j + 1][2]) {
              Xref[0] = Y[j][0];
              Xref[1] = Y[j][1];
            } else {
              Xref[0] = Y[j + 1][0];
              Xref[1] = Y[j + 1][1];
            }
          }
          v = vector(X[2][0], X[2][1], Xref[0], Xref[1]);
          if (*(v + 1) > 0) {
            forward();
            car_direction = 2;
          }

          else if (*(v) >= 0) {
            right();
            car_direction = 1;
          }

          else {
            left();
            car_direction = 3;
          }

          break;
        case 3:
          inputY(X[2][0] - 1, X[2][1], 0, 2);
          inputY(X[2][0] , X[2][1] + 1, 1, 2);
          inputY(X[2][0] , X[2][1] - 1, 2, 2);
          for (int j = 0; j < 2; j++) {
            if (Y[j][2] > Y[j + 1][2]) {
              Xref[0] = Y[j][0];
              Xref[1] = Y[j][1];
            } else {
              Xref[0] = Y[j + 1][0];
              Xref[1] = Y[j + 1][1];
            }
          }
          v = vector(X[2][0], X[2][1], Xref[0], Xref[1]);
          if (*(v) < 0) {
            forward();
            car_direction = 3;
          }

          else if (*(v + 1) >= 0) {
            right();
            car_direction = 2;
          }

          else {
            left();
            car_direction = 4;
          }

          break;
        case 4:
          inputY(X[2][0] + 1, X[2][1], 0, 2);
          inputY(X[2][0] - 1, X[2][1], 1, 2);
          inputY(X[2][0] , X[2][1] - 1, 2, 2);
          for (int j = 0; j < 2; j++) {
            if (Y[j][2] > Y[j + 1][2]) {
              Xref[0] = Y[j][0];
              Xref[1] = Y[j][1];
            } else {
              Xref[0] = Y[j + 1][0];
              Xref[1] = Y[j + 1][1];
            }
          }
          v = vector(X[2][0], X[2][1], Xref[0], Xref[1]);
          if (*(v + 1) < 0) {
            forward();
            car_direction = 4;
          }

          else if (*(v) >= 0) {
            left();
            car_direction = 1;
          }

          else {
            right();
            car_direction = 3;
          }

          break;
      }
      change(X[0], X[1], X[2], Xref, sensorvalues);
      sensorRead();
    }


  }
  

}
double getppm (int i) {
  Serial.println("--------------------------------------getppm---");
  double sensor_volt[4]={0};
  double RS_gas[4]={0}; // Get value of RS in a GAS
  double ratio[4]={0}; // Get ratio RS_GAS/RS_air
  double c[4]={0};
  double d[4]={0};
  double a = -58.75;
  double b = -35.25;
  const double calculateL =100.0; 
  double ppm[4]={0};
  double sensorValue[4]={0};// = analogRead(A0);
      for (int j = 0; j < 100; j++) {
      sensorValue[i] += analogRead(mq3[i]);
      delay (delaytime);
    }
    sensorValue[i] /= 100.0;
    Serial.print("--(");
    Serial.print(i);
    Serial.print(")--sensorValue=");
   Serial.println(sensorValue[i]);
    sensor_volt[i] = sensorValue[i] / 1024 * 5.0;
    RS_gas[i] = (5.0 - sensor_volt[i]) / sensor_volt[i];
    ratio[i] = RS_gas[i] / R0[i];
    Serial.print("ratio=");
     Serial.println(ratio[i]);
    d[i] = log10(ratio[i])*calculateL;  
    c[i] = (d[i] - b) / a;
    ppm[i] = pow(10, c[i]) * enlarge;
  Serial.print("ppm(should be 2.14~132535.18)");
  Serial.print(i);
  Serial.print("=");
  Serial.println(ppm[i]);
  if(ppm[i] == NAN)
  {
    ppm[i]=3.0;
    Serial.println(ppm[i]);
    }
    
  return ppm[i];
  }
void sensorRead() {
  Serial.println("----------------------------------sensorRead---");
  for (int i = 0; i < 4; i++) {
    sensorvalues[2][i] = getppm(i);

    Serial.print("sensor");
    Serial.print(i);
    Serial.print("=");
    Serial.println(sensorvalues[2][i]);
  }
  getC(sensorvalues, 2);
  Serial.print("C(X3)=");
  Serial.println(X[2][2]);
}
void change(double X0[], double X1[], double X2[], double Xnext[], double sensors[][4]) {
  X0[0] = X1[0];
  X0[1] = X1[1];
  X0[2] = X1[2];
  X1[0] = X2[0];
  X1[1] = X2[1];
  X1[2] = X2[2];
  X2[0] = Xnext[0];
  X2[1] = Xnext[1];
  for (int i = 0; i < 4; i++) {
    sensors[0][i] = sensors[1][i];
    sensors[1][i] = sensors[2][i];
    sensors[2][i] = 0;
  }

}
double *vector(double x1,double y1,double x2,double y2) {
  *v = x2 - x1;
  *(v + 1) = y2 - y1;
  Serial.println("----------------------------------------vector-");  
  Serial.print("(");
      Serial.print(x1);
      Serial.print("  ,  ");
      Serial.print(y1);
      Serial.println(")");
      Serial.print("(");
      Serial.print(x2);
      Serial.print("  ,  ");
      Serial.print(y2);
      Serial.println(")");
  Serial.print("vector=");
  Serial.print(*v);
  Serial.print(",");
  Serial.println(*(v + 1));
  
  return v;
}
double getC(double sensorvalues[][4], int now) {//濃度計算
  Serial.println("----------------------------------------getC---");
  double diff[4];
  double sum = 0.0;
  for (int i = 0; i < 4; i++) {
    Serial.print("sensorvalues");
    Serial.print(i);
    Serial.print("=");
    Serial.println(sensorvalues[now][i]);
    diff[i] = fabs(sensorvalues[now][i] - sensorvalues[now - 1][i]);
    Serial.print("diff");
    Serial.print(i);
    Serial.print("=");
    Serial.println(diff[i]);
    sum += fabs(sensorvalues[now][i] - sensorvalues[now - 1][i]);
  }
  if (sum == 0) {
    sum = 1;
  }
  Serial.print("sum=");
  Serial.println(sum);
  
  X[now][2] = (diff[0] / sum * sensorvalues[now][0]) +
              (diff[1] / sum * sensorvalues[now][1]) +
              (diff[2] / sum * sensorvalues[now][2]) +
              (diff[3] / sum * sensorvalues[now][3]) ;
  Serial.print("X=");
  Serial.println(X[now][2]);
}
double distance(int x2, int y2, int x1, int y1) { //Y(x2,y2) X(x1,y1)
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)) / RobotPace;
}
float FYX( int x2, int y2, int x1, int y1, double C) { //F(Y,X)=C(X)/Distance(Y,X)
  double D = distance(x2, y2, x1, y1);
  return C / D;
}
double P1(int x2, int y2) {//P1(Y)
  double Xsort[3][3]={0};//[X C]*3
  for (int i = 0; i < 3; i++) {
    if (max(X[0][2], (max(X[1][2], X[2][2]))) == X[i][2]) {
      Xsort[0][2] = X[i][2];
      Xsort[0][0] = X[i][0];
      Xsort[0][1] = X[i][1];
    } else if (min(X[0][2], (min(X[1][2], X[2][2]))) == X[i][2]) {
      Xsort[2][2] = X[i][2];
      Xsort[2][0] = X[i][0];
      Xsort[2][1] = X[i][1];
    } else {
      Xsort[1][2] = X[i][2];
      Xsort[1][0] = X[i][0];
      Xsort[1][1] = X[i][1];
    }
  }
  double sum = fabs(Xsort[0][2] - Xsort[1][2]) + fabs(Xsort[1][2] - Xsort[2][2]) + fabs(Xsort[0][2] - Xsort[2][2]);
  if (sum==0){
  sum=1;
  }
  double lambda0 = fabs(Xsort[0][2] - Xsort[1][2]) / sum;
  double lambda1 = fabs(Xsort[1][2] - Xsort[2][2]) / sum;
  double lambda2 = fabs(Xsort[0][2] - Xsort[2][2]) / sum;
  double result = lambda0 * (FYX(x2, y2, Xsort[0][0], Xsort[0][1], Xsort[0][2]) - FYX(x2, y2, Xsort[1][0], Xsort[1][1], Xsort[1][2]))
                  + lambda1 * (FYX(x2, y2, Xsort[1][0], Xsort[1][1], Xsort[1][2]) - FYX(x2, y2, Xsort[2][0], Xsort[2][1], Xsort[2][2]))
                  + lambda2 * (FYX(x2, y2, Xsort[0][0], Xsort[0][1], Xsort[0][2]) - FYX(x2, y2, Xsort[2][0], Xsort[2][1], Xsort[2][2]));
  Serial.print("P1=");
  Serial.println(result);
  return result;
}
double P2(int x2, int y2) {//P2(Y)
  double D = min(distance(x2, y2, X[1][0], X[1][1]), distance(x2, y2, X[2][0], X[2][1]));
  double delta = (sensorvalues[1][2] + sensorvalues[1][3] + sensorvalues[2][2] + sensorvalues[2][3]) / 4
                 - (sensorvalues[1][0] + sensorvalues[1][1] + sensorvalues[2][0] + sensorvalues[2][1]) / 4;
  double eta0 = (float)fabs(X[2][2] - X[1][2]) / (fabs(X[2][2] - X[1][2]) + fabs(delta));
  double eta1 = fabs(delta) / (fabs(X[2][2] - X[1][2]) + fabs(delta));
  int sign;
  if (X[2][2] - X[1][2] > 0)
    sign = 1;
  else if (X[2][2] - X[1][2] < 0)
    sign = -1;
  else sign = 0;
  int sign2;
  if (((X[2][0] - X[1][0]) * (y2 - X[1][1])) - ((X[2][1] - X[1][1]) * (x2 - X[1][0])) > 0)
    sign2 = 1;
  else if (((X[2][0] - X[1][0]) * (y2 - X[1][1])) - ((X[2][1] - X[1][1]) * (x2 - X[1][0])) < 0)
    sign2 = -1;
  else
    sign2 = 0;
  double result = eta0 * fabs((FYX(x2, y2, X[2][0], X[2][1], X[2][2]) - (FYX(x2, y2, X[1][0], X[1][1], X[1][2])))) * sign + eta1 * delta * sign2 / D;
  Serial.print("P2=");
  Serial.println(result);
  return result;
}
double P(int x, int y) {//P(Y)
  double distance1 = distance(x, y, X[0][0], X[0][1]);
  //Serial.print("distance=");
  //Serial.println(distance1);
  double distance2 = distance(x, y, X[1][0], X[1][1]);
  double D = min(distance1, distance2 );
  //Serial.print("D=");
  //Serial.println(D);
  double delta = (sensorvalues[0][0] + sensorvalues[0][1] + sensorvalues[1][0] + sensorvalues[1][1]) / 4
                - (sensorvalues[0][3] + sensorvalues[0][2] + sensorvalues[1][3] + sensorvalues[1][2]) / 4;
  //Serial.print("delta=");
  //Serial.println(delta);
  double eta0 = fabs(X[2][2] - X[0][2]) / (fabs(X[1][2] - X[0][2]) + fabs(delta));
  //Serial.print("eta0=");
  //Serial.println(eta0);
  double eta1 = fabs(delta) / (fabs(X[1][2] - X[0][2]) + (float)fabs(delta));
  //Serial.print("eta1=");
  //Serial.println(eta1);
  int sign;   //byte?
  if (X[1][2] - X[0][2] > 0)
    sign = 1;
  else if (X[1][2] - X[0][2] < 0)
    sign = -1;
  else sign = 0;

  int sign2;
  if (((X[1][0] - X[0][0]) * (y - X[0][1])) - ((X[1][1] - X[0][1]) * (x - X[0][0])) > 0)
    sign2 = 1;
  else if (((X[1][0] - X[0][0]) * (y - X[0][1])) - ((X[1][1] - X[0][1]) * (x - X[0][0])) < 0)
    sign2 = -1;
  else
    sign2 = 0;
  double result = eta0 * fabs(X[1][2] / distance1 - X[0][2] / distance2) * sign + eta1 * delta * sign2 / D;
  Serial.print("P=");
  Serial.println(result);
  return result;
}
void inputY(int x, int y, int num, int predictF) {
  for (int i = 0; i < 3; i++) {
    if (x == X[i][0] && y == X[i][1])
      return 0;
  Y[num][0] = x;
  Y[num][1] = y;
  }
  switch (predictF) {
    case 0: Y[num][2] = P(x, y);  break;
    case 1: Y[num][2] = P1(x, y); break;
    case 2: Y[num][2] = P2(x, y); break;
  }

}
long dodge() {
  long duration, cm;
  digitalWrite(2, LOW);
  delayMicroseconds(5);
  digitalWrite(2, HIGH);     // 給 Trig 高電位，持續 10微秒
  delayMicroseconds(10);
  digitalWrite(2, LOW);
  pinMode(3, INPUT);             // 讀取 echo 的電位
  duration = pulseIn(3, HIGH);   // 收到高電位時的時間
  cm = (duration / 2) / 29.1;       // 將時間換算成距離 cm 或 inch
  delay(250);
  return cm;
}
void forward() {
  for (int s = 0; s < 200; s++)
  {
    Lstepper.step(1);
    Rstepper.step(1);
  };
}
void back() {
  for (int s = 0; s < 200; s++)
  {
    Lstepper.step(-1);
    Rstepper.step(-1);
  };
}
void left() {
  for (int s = 0; s < 200; s++)
  {
    Lstepper.step(-1);
    Rstepper.step(1);
  };
  forward();
}
void right() {
  for (int s = 0; s < 200; s++)
  {
    Lstepper.step(1);
    Rstepper.step(-1);
  };
  forward();
}
