#define MOTOR_PWM_PIN 6
#define MOTOR_DIR_PIN 8
#define ENCODER_PIN 2

// PID gains (start conservative, tune later)
double Kp = 0.8;
double Ki = 0.2;
double Kd = 0.5;

double setpoint = 200.0;
double input = 0.0;
double output = 0.0;
double lastInput = 0.0;
double integral = 0.0;

const double dt = 0.05;  // 50ms loop time

// For filtered derivative
double filteredDerivative = 0.0;
const double alpha = 0.7;

// For RPM smoothing
#define FILTER_SIZE 5
double rpmBuffer[FILTER_SIZE];
int rpmIndex = 0;

// Simulated encoder count (replace with your actual encoder implementation)
volatile long encoderCount = 0;
unsigned long lastEncoderTime = 0;

void encoderISR() {
  encoderCount++;
}

// Your RPM measurement function
double readRPM() {
  static long lastCount = 0;
  static unsigned long lastTime = 0;

  unsigned long now = millis();
  long count = encoderCount;

  double rpm = 0.0;
  if (lastTime > 0) {
    double deltaTime = (now - lastTime) / 1000.0;
    if (deltaTime > 0) {
      rpm = (count - lastCount) * (60.0 / (20.0 * deltaTime));  // adjust 20.0 to your encoder PPR
    }
  }

  lastTime = now;
  lastCount = count;
  return rpm;
}

// Moving average filter for RPM
double readFilteredRPM() {
  double raw = readRPM();
  rpmBuffer[rpmIndex] = raw;
  rpmIndex = (rpmIndex + 1) % FILTER_SIZE;

  double sum = 0.0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += rpmBuffer[i];
  }
  return sum / FILTER_SIZE;
}

void setup() {
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  Serial.begin(9600);

  // Initialize RPM buffer
  for (int i = 0; i < FILTER_SIZE; i++) {
    rpmBuffer[i] = 0.0;
  }

  lastEncoderTime = millis();
}

void loop() {
  input = readFilteredRPM();

  // PID calculations
  double error = setpoint - input;

  // Integral with anti-windup
  integral += Ki * error * dt;
  integral = constrain(integral, -100, 100);

  // Derivative (filtered)
  double rawDerivative = (input - lastInput) / dt;
  filteredDerivative = alpha * filteredDerivative + (1 - alpha) * rawDerivative;

  // PID output
  output = Kp * error + integral - Kd * filteredDerivative;
  output = constrain(output, 0, 255);

  // Apply output to motor
  analogWrite(MOTOR_PWM_PIN, (int)output);

  // For debugging
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" RPM, Actual: ");
  Serial.print(input);
  Serial.print(" RPM, PWM: ");
  Serial.println(output);

  // Prepare for next loop
  lastInput = input;

  delay(50);  // Sampling time ~50ms
}
