# Motor_Speed_Controller-pid_motor_control.ino
// Motor Speed Controller with PID ⚙️
// Example Arduino sketch for PID motor control

int motorPin = 9;
int sensorPin = A0;

float Kp = 2.0, Ki = 0.5, Kd = 1.0;
float setPoint = 200, input, output, error, prevError, integral;

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  input = analogRead(sensorPin);
  error = setPoint - input;
  integral += error;
  float derivative = error - prevError;
  output = Kp*error + Ki*integral + Kd*derivative;

  analogWrite(motorPin, constrain(output, 0, 255));

  Serial.print("Input: "); Serial.print(input);
  Serial.print(" Output: "); Serial.println(output);

  prevError = error;
  delay(100);
}
