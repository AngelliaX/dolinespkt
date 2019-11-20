//File lấy dữ liệu
//Số đc lấy từ trái sang phảiic
#define motor1 2
#define motor2 4
#define motor3 7
#define motor4 9
const int motorlow[4] = {3, 5, 6, 8};
const int trig = 5;     // chân trig của HC-SR04
const int echo = 6;     // chân echo của HC-SR04

int trungbinh = 340;
int nguongmin = 300; // set de tat ca deu = 300 khi ko co line
const int qtro[5] = {A0, A1, A2, A3, A4};
int ktraqtro[5] = {0, 0, 0, 0, 0};
int locqtro[5] = {0, 0, 0, 0, 0};
int tanggiam[5] = {0, 0, 0, 0, 0};
bool isRun = false;
int timer = 0;
float fix_sp = 2;
//---------------------------------------------------------------------

void setup() {
  //pinMode(trig, OUTPUT);  // chân trig sẽ phát tín hiệu
  //pinMode(echo, INPUT);   // chân echo sẽ nhận tín hiệu

  Serial.begin(9600);
  for (int i = 0; i < 5; i++) {
    pinMode(qtro[i], INPUT);
  }
}
//--------------------------------------------------------------------
bool a = false;
void loop() {
  //ktravatcan();
  //  analogWrite(motor1,200);
  //  analogWrite(motor3,200);
  //  analogWrite(motorlow[1],200);
  //  analogWrite(motorlow[3],200);

  //   analogWrite(motor2, 100);
  //  analogWrite(motor4, 100);

  Getlocquangtro();
  digital();

  if (!a) {
    laytanggiam();
    a = true;
    //delay(1000);
  }
  laydulieu();

  //  //delay(100);
}
//----------Ktra cac kieu-------------------------------------------------------
void GetSensorAnalog() {
  for (int i = 0; i < 5; i++) {
    Serial.print(analogRead(qtro[i]));
    Serial.print("       ");
  }

  Serial.println();
  //delay(10);
}
void Gettanggiam() {
  for (int i = 0; i < 5; i++) {
    Serial.print(tanggiam[i]);
    Serial.print("  ");
  }

  Serial.println();
}
void Getlocquangtro() {
  for (int i = 0; i < 5; i++) {
    locqtro[i] = analogRead(qtro[i]) - tanggiam[i];
    Serial.print(locqtro[i]);
    Serial.print("  ");
  }
  //Serial.println();
}
void digital() {
  for (int i = 0; i < 5; i++) {
    if (locqtro[i] > trungbinh) {
      ktraqtro[i] = 1;
    } else {
      ktraqtro[i] = 0;
    };
  }
  for (int i = 0; i < 5; i++) {
    Serial.print(ktraqtro[i]);
    Serial.print("  ");
  }
  Serial.println();
}
void pidinfo(int x, int y, int pid) {
  Serial.print("X: ");
  Serial.println(x);
  Serial.print("Y: ");
  Serial.println(y);
  Serial.print("Pid: ");
  Serial.println(pid);
}
//-------------------------------------------------------------------

//--------------------------------------------------------------
long long int sys_start = 0 ;
void ktravatcan() {
  unsigned long duration; // biến đo thời gian
  int distance;           // biến lưu khoảng cách

  digitalWrite(trig, 0);  // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(trig, 1);  // phát xung từ chân trig
  delayMicroseconds(5);   // xung có độ dài 5 microSeconds
  digitalWrite(trig, 0);  // tắt chân trig
  duration = pulseIn(echo, HIGH);
  distance = int(duration / 2 / 29.412);
  if (distance < 15) {
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    isRun = false;
    sys_start = millis();
  } else {
    laydulieu();
  }
}

//-------------------------------------------------------------------
int pid;

bool Line[5] = {false};

int soluongqtrophiatrenline = 0;
long long int timecheck = 0;
bool done = true;
bool cuaphai = false;
bool cuatrai = false;
long long int delayed = 0;
void LinePIDFilter() { //PID control in line tracking mode
  static double kP = 10, kI = 0, kD = 0; //69 la max //lqd 67 or 62-59//0.5 -15
  static double gain = 100;
  static double prev_error = 0, prev_I = 0;
  double p = 0, i = 0, d = 0, pid_value;

  p = xulyloi();
  // Serial.println(p);
  i = p + prev_I;
  d = p - prev_error;

  pid_value = kP * p + kI * i + kD * d;
  pid = pid_value;
  prev_I = i;
  prev_error = p;
  pid_value *= gain / 100;

  float ratio = 0;
  float x = 255 + pid;
  float y = 255 - pid;

  if (x > 255 || y > 255) {
    if (x < y) {
      ratio = 255 / y;
    } else {
      ratio = 255 / x;
    }
    x *= ratio;
    y *= ratio;
  }

  if (!isRun) {
    if (millis() - sys_start <= 500) {
      x = x * (millis() - sys_start + 200) / 700;
      y = y * (millis() - sys_start + 200) / 700;

    } else {
      isRun = true;
    }
  }

  constrain(x, 0, 255);
  constrain(y, 0, 255);

  if (soluongqtrophiatrenline > 2) {
    if (!Line[0] && Line[4]) {
      Serial.println("------------------cua phai----------------");
      timecheck = millis();
      done = false;
      cuaphai = true;
    } else if (Line[0] && !Line[4]) {
      Serial.println("------------------cua trai----------------");
      timecheck = millis();
      done = false;
      cuatrai = true;
    }
} else {
  delayed = millis();
}
Serial.println("alo");
if (!done ) {
  if (cuaphai) {
    if (millis() - timecheck > 100) {
      if (Line[2] || Line[3]) {
        done = true;
        cuaphai = false;
      }
      analogWrite(motor1, 140);
      analogWrite(motor3, 140);
      analogWrite(motorlow[0], 0);
      analogWrite(motorlow[2], 0);
      analogWrite(motorlow[1], 140);
      analogWrite(motorlow[3], 140);
      analogWrite(motor2, 0);
      analogWrite(motor4, 0);
    }
    Serial.println("cua phai");
  } else if (cuatrai) {
    if (millis() - timecheck > 100) {
      if (Line[1] || Line[2]) {
        done = true;
        cuatrai = false;
      }
      analogWrite(motor2, 140);
      analogWrite(motor4, 140);
      analogWrite(motorlow[1], 0);
      analogWrite(motorlow[3], 0);
      analogWrite(motorlow[0], 140);
      analogWrite(motorlow[2], 140);
      analogWrite(motor1, 0);
      analogWrite(motor3, 0);
    }
    Serial.println("cua trai");

  }
  return false;
}
if (false) {
} else {
  if (y < 250 ) { //y<250
    analogWrite(motorlow[1], (int)y / fix_sp - 25);
    analogWrite(motorlow[3], (int)y / fix_sp - 25);
    analogWrite(motor2, 0);
    analogWrite(motor4, 0);
  } else {
    analogWrite(motorlow[1], 0);
    analogWrite(motorlow[3], 0);
    analogWrite(motor2, (int)y / fix_sp);
    analogWrite(motor4, (int)y / fix_sp);
  }
  if (x < 250 ) { //x<250
    analogWrite(motorlow[0], (int)x / fix_sp - 25);
    analogWrite(motorlow[2], (int)x / fix_sp - 25);
    analogWrite(motor1, 0);
    analogWrite(motor3, 0);
  } else {
    analogWrite(motorlow[0], 0);
    analogWrite(motorlow[2], 0);
    analogWrite(motor1, (int)x / fix_sp);
    analogWrite(motor3, (int)x / fix_sp);

  }
}
}
void laytanggiam() {
  for (int i = 0; i < 5; i++) {
    tanggiam[i] = analogRead(qtro[i]) - nguongmin;
    Serial.println(tanggiam[i]);
  }
}
void laydulieu() {

  // for (int i = 0; i < 5; i++) {
  //ktraqtro[i] = 0;
  //}

  for (int i = 0; i < 5; i++) {
    locqtro[i] = analogRead(qtro[i]) - tanggiam[i];
  }

  for (int i = 0; i < 5; i++) {
    if (locqtro[i] > trungbinh) {
      ktraqtro[i] = 1;
    } else {
      ktraqtro[i] = 0;
    };
  }
  LinePIDFilter();
}



//---------------------------------------------------------------------------------


int xulyloi() {
  //Kiểm tra số quang trở phía trên line
  for (int i = 0; i < 5; i++) {
    Line[i] = false;
  }
  soluongqtrophiatrenline  = 0;
  for (int i = 0; i < 5; i++) {
    if (ktraqtro[i]) {
      soluongqtrophiatrenline += 1;
    }
  }
  /*
    if(soluongqtrophiatrenline >=3){
    fix_sp = 4;
    44ntln();
    Serial.print(">3");
    Serial.println();
    }else{
     fix_sp = 1.1;
    }*/
  //Chuyển từ 0001000 sang false,true
  for (int i = 0; i < 5; i++) {
    if (ktraqtro[i] == 1) {
      Line[i] = true;
    }
  }

  //Xác định lỗi
  static int prev_error = 0;
  switch (soluongqtrophiatrenline) {
    case 0: {
        //        if (prev_error == 0) {
        //          return 0;
        //        } else if (prev_error == 4 || prev_error == 5) {
        //          prev_error = 5;
        //          return 5;
        //        } else if (prev_error == -4 || prev_error == -5) {
        //          prev_error = -5;
        //          return -5;
        //        }
        //        else
        return prev_error;
        break;
      }
    case 1: {
        for (int i = 0; i < 5; i++) {
          if (Line[i]) {
            switch (i) {
              case 0: {
                  prev_error = -4;
                  return -4;
                  break;
                }
              case 1: {
                  prev_error = -2;
                  return -2;
                  break;
                }
              case 2: {
                  prev_error = 0;
                  return 0;
                  break;
                }
              case 3: {
                  prev_error = 2;
                  return 2;
                  break;
                }
              case 4: {
                  prev_error = 4;
                  return 4;
                  break;
                }
              default: {
                  return prev_error;
                  break;
                }
            }
            break;
          }
        }
        break;
      }
    case 2: {
        if (Line[0] && Line[1]) {
          prev_error = -3;
          return -3;
        } else if (Line[1] && Line[2]) {
          prev_error = -1;
          return -1;
        } else if (Line[2] && Line[3]) {
          prev_error = 1;
          return 1;
        } else if (Line[3] && Line[4]) {
          prev_error = 3;
          return 3;
        } else return prev_error;
        break;
      }
    case 3: {
        if (Line[0] && Line[1] && Line[2]) {
          if (prev_error == 1) {
            prev_error = 1;
            return 1;
          }
          prev_error = -1;
          return -1;
        } else if (Line[1] && Line[2] && Line[3]) {
          prev_error = 0;
          return 0;
        } else if (Line[2] && Line[3] && Line[4]) {
          if (prev_error == -1) {
            prev_error = -1;
            return -1;
          }
          prev_error = 1;
          return 1;
        } else return prev_error;
        break;
      }
    case 4: {
        if (Line[0] && Line[1] && Line[2] && Line[3]) {
          prev_error = -4;
          return -4;
        }
        else if (Line[1] && Line[2] && Line[3] && Line[3]) {
          prev_error = 4;
          return 4;
        }
        //                else if(Line[0] && Line[1] && Line[2] && Line[4]){
        //                  prev_error = -4;
        //                  return -4;
        //                }else if(Line[0] && Line[3] && Line[2] && Line[4]){
        //                  prev_error = 4;
        //                  return 4;
        //                }
        else {
          return prev_error;
        }
        //return prev_error;
        break;
      }
    case 5: {
        prev_error = 0;
        return 0;
      }
    default: {
        return prev_error;
      }
  }
  Serial.print(prev_error);
  Serial.println();
}
