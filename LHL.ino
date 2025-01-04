
const int enaLeftPin = 5;   // Enable pin for the left motor
const int in1LeftPin = 6;   // Input 1 pin for the left motor
const int in2LeftPin = 7;   // Input 2 pin for the left motor

// Motor control pins for the right motor
const int enaRightPin = 10;  // Enable pin for the right motor
const int in1RightPin = 8;   // Input 1 pin for the right motor
const int in2RightPin = 9;   // Input 2 pin for the right motor

// PID constants
float kp = 2.75;  // Proportional constant
float ki = 0;  // Integral constant
float kd = 0;  // Derivative constant
//
int maxspeed = 110;
int turnspeed = 120;
int previousError = 0;
int I = 0;
int error;
int f, r1, r2, r3, r4, r5, r6, r7, r8, endp;
int actual, output, D, P;
int leftSpeed;
int rightSpeed;
int enaLeft;
int enaRight;

void readIR() {
  r1 = analogRead(A1);//most right
  r2 = analogRead(A2);
  r3 = analogRead(A3);
  r4 = analogRead(A4);
  r5 = analogRead(A8);
  r6 = analogRead(A9);
  r7 = analogRead(A10);
  r8 = analogRead(A11);
  f = analogRead(A0);
  endp = analogRead(A12);

  r1 = (r1 > 450) ? 1 : 0;
  r2 = (r2 > 450) ? 1 : 0;
  r3 = (r3 > 450) ? 1 : 0;
  r4 = (r4 > 450) ? 1 : 0;
  r5 = (r5 > 450) ? 1 : 0;
  r6 = (r6 > 450) ? 1 : 0;
  r7 = (r7 > 450) ? 1 : 0;
  r8 = (r8 > 450) ? 1 : 0;
  f = (f > 300) ? 1 : 0;
  endp = (endp > 450) ? 1 : 0;

}

void forward() {

  actual = -40 * r1 - 30 * r2 - 20 * r3 - 10 * r4 + 10 * r5 + 20 * r6 + 30 * r7 + 40 * r8;
  error = 0 - actual;

  P = kp * error;
  I += ki * error;
  D = kd * (error - previousError);
  output = P + I + D;
  leftSpeed = constrain(maxspeed + output, 0, 255);
  rightSpeed = constrain(maxspeed - output, 0, 255);

  digitalWrite(6, HIGH);//left
  digitalWrite(7, LOW);
  digitalWrite(9, LOW);//right
  digitalWrite(8, HIGH);
  analogWrite(5, leftSpeed);
  analogWrite(10, rightSpeed);

  previousError = error;
}
void stoop() {
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
  delay(200);
}

void left() {
  ///////////////
  digitalWrite(6, LOW);//left
  digitalWrite(7, HIGH);
  digitalWrite(9, LOW);//right
  digitalWrite(8, HIGH);
  analogWrite(enaRightPin, turnspeed);
  analogWrite(enaLeftPin, 0 );
  delay(150);
  do {
    digitalWrite(6, LOW);//left
    digitalWrite(7, HIGH);
    digitalWrite(9, LOW);//right
    digitalWrite(8, HIGH);
    analogWrite(enaRightPin, turnspeed);
    analogWrite(enaLeftPin, 0 );
    readIR();
  } while ( f == 0  );
}

void right() {
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);
  analogWrite(enaRightPin, 0);
  analogWrite(enaLeftPin, turnspeed + 30);
  delay(150);
  do {
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(9, HIGH);
    digitalWrite(8, LOW);
    analogWrite(enaRightPin, 0);
    analogWrite(enaLeftPin, turnspeed + 30);
    readIR();
  } while ( f == 0  );
}

void back() {
  digitalWrite(6, HIGH);//left
  digitalWrite(7, LOW);
  digitalWrite(9, LOW);//right
  digitalWrite(8, HIGH);
  analogWrite(enaRightPin, turnspeed);
  analogWrite(enaLeftPin, turnspeed );
  delay(150);
  digitalWrite(6, LOW);//left
  digitalWrite(7, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  analogWrite(enaRightPin, turnspeed );
  analogWrite(enaLeftPin, turnspeed + 15);
  delay(100);
  do {
    digitalWrite(6, LOW);//left
    digitalWrite(7, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(8, HIGH);
    analogWrite(enaRightPin, turnspeed / 1.3);
    analogWrite(enaLeftPin, turnspeed / 1.3 + 15);
    readIR();
  } while ( f == 0  );
}
void setup() {
  // Initialize motor control pins as outputs
  pinMode(enaLeftPin, OUTPUT);
  pinMode(in1LeftPin, OUTPUT);
  pinMode(in2LeftPin, OUTPUT);
  pinMode(enaRightPin, OUTPUT);
  pinMode(in1RightPin, OUTPUT);
  pinMode(in2RightPin, OUTPUT);
  //
  Serial.begin(9600);
}

void loop() {

  readIR();

  // commented this forthe sake of testing

  /*  if ( r1 == 1 && r2 == 1 && r3 == 1 && r4 == 1 && r5 == 1 && r6 == 1 && r7 == 1 && r8 == 1 && f == 1) // // commented this forthe sake of testing
    {
      stoop();
    }
    else if (r1 == 0 && r2 == 0 && r3 == 0 && r6 == 1 && r7 == 1 && r8 == 1  && endp == 0) //left3 are one && right3 are zero
    {
      stoop();
      left();
    }
    else if (f == 1 ||  r3 == 1 || r4 == 1 || r5 == 1 || r6 == 1 && endp == 0) {
      forward();
    }
    else if (r1 == 0 && r2 == 0 && r3 == 0 && r5 == 1 && r6 == 1 && r7 == 1 && r8 == 1 && f == 1 && endp == 0) {
      stoop();
      right();
    }
    else {
      stoop();
      back();

    }
  */

  if ( r1 == 1 && r2 == 1 && r3 == 1 && r4 == 1 && r5 == 1 && r6 == 1 && r7 == 1 && r8 == 1 && f == 1 && endp == 1)
  {
    stoop();
  }
    else if ( r6 == 1 && r7 == 1 && r8 == 1 ) 
  {
    stoop();
    left();
    
  }
  
  else if (f == 1 || r4 == 1 || r5 == 1 ) {
    forward();
    //push forward
  }
else if (r1 == 1 && r2 == 1 && r3 == 1 )
  {
    stoop();
    right();
    //push r
  }

  else if (r1 == 0 && r2 == 0 && r3 == 0 && r4 == 0 && r5 == 0 && r6 == 0 && r7 == 0 && r8 == 0 && f == 0) {
    stoop();
    back();
    //u turn 
    //pop
  }



}
