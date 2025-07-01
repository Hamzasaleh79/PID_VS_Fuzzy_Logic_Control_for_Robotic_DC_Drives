#define MOTOR_PWM_PIN 6
#define MOTOR_DIR_PIN 8
#define ENCODER_PIN 2

const int TICKS_PER_REVOLUTION = 20; // Verify your encoder's PPR

// System state
volatile unsigned long encoderCount = 0;
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 75; // 75ms for faster response
double measuredRPM = 0;
double targetRPM = 200.0;

// Moving average filter for RPM (5 samples)
double rpmBuffer[5] = {0, 0, 0, 0, 0};
int bufferIndex = 0;

// Performance monitoring
double bestPWM = 0;
unsigned long settleStartTime = 0;
unsigned long lastBestPWMReset = 0;
const unsigned long BEST_PWM_RESET_INTERVAL = 10000; // Reset bestPWM every 10s if unsettled
const double settleThreshold = 3.0; // ±3 RPM considered "settled"
const unsigned long settleTime = 1500; // 1.5 seconds to consider settled

// Output constraints
const int MIN_PWM = 70;  // Minimum PWM to overcome friction
const int MAX_PWM = 220; // Limited to protect motor

// Encoder debouncing
unsigned long lastEncoderTime = 0;
const

unsigned long DEBOUNCE_TIME = 5; // 5ms for noise rejection
const unsigned long MIN_COUNT = 2; // Minimum encoder count for valid RPM

// Fuzzy logic variables
double error = 0;
double lastError = 0;
double deltaError = 0;
double integral = 0;
double output = 0;
const double KI = 0.8; // Increased integral gain for steady-state error correction
const double INTEGRAL_LIMIT = 30.0; // Anti-windup limit

// Fuzzy membership functions (triangular: a, b, c)
struct FuzzySet {
  double a, b, c;
};

// Membership functions for error, deltaError, and output
FuzzySet errorSets[5] = {
  {-60, -45, -30}, // NL
  {-45, -15, 0},   // NS
  {-10, 0, 10},    // ZE
  {0, 15, 45},     // PS
  {30, 45, 60}     // PL
};

FuzzySet deltaErrorSets[5] = {
  {-60, -45, -30}, // NL
  {-45, -15, 0},   // NS
  {-10, 0, 10},    // ZE
  {0, 15, 45},     // PS
  {30, 45, 60}     // PL
};

FuzzySet outputSets[5] = {
  {-30, -25, -20}, // NL
  {-25, -10, 0},   // NS
  {-10, 0, 10},    // ZE
  {0, 10, 25},     // PS
  {20, 25, 30}     // PL
};

// Fuzzy rule base (5x5)
int rules[5][5] = {
  {4, 4, 0, 0, 0}, // NL: PL, PL, NL, NL, NL
  {4, 3, 0, 0, 0}, // NS: PL, PS, NL, NL, NL
  {3, 2, 2, 0, 0}, // ZE: PS, ZE, ZE, NL, NL
  {2, 1, 0, 0, 0}, // PS: ZE, NS, NL, NL, NL
  {1, 0, 0, 0, 0}  // PL: NS, NL, NL, NL, NL
};

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, HIGH);

  // Encoder setup with interrupt
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  // Initial motor test with limited power
  Serial.println("Running motor test at 40% power...");
  analogWrite(MOTOR_PWM_PIN, 102); 
  delay(1500);
  analogWrite(MOTOR_PWM_PIN, 0);
  delay(500);
  
  Serial.println("Fuzzy Logic Control Initialized");
  Serial.print("Target RPM: "); Serial.println(targetRPM);
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleTime >= sampleInterval) {
    // Get encoder count safely
    noInterrupts();
    unsigned long count = encoderCount;
    encoderCount = 0;
    interrupts();

    // Calculate RPM and apply moving average filter
    double rawRPM = (count >= MIN_COUNT) ? (count * 60000.0) / (TICKS_PER_REVOLUTION * sampleInterval) : 0;
    rawRPM = constrain(rawRPM, 0, 500); // Limit to realistic maximum
    rpmBuffer[bufferIndex] = rawRPM;
    bufferIndex = (bufferIndex + 1) % 5;
    double sum = 0;
    for(int i = 0; i < 5; i++) sum += rpmBuffer[i];
    measuredRPM = sum / 5;

    // Fuzzy Logic Control
    error = targetRPM - measuredRPM;
    deltaError = error - lastError;

    // Integral term (with anti-windup)
    if (abs(error) < 50.0) { // Wider range for integral action
      integral += error * (sampleInterval / 1000.0);
      integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }

    // Fuzzification
    double errorMemberships[5];
    double deltaErrorMemberships[5];
    for(int i = 0; i < 5; i++) {
      errorMemberships[i] = getMembership(error, errorSets[i]);
      deltaErrorMemberships[i] = getMembership(deltaError, deltaErrorSets[i]);
    }

    // Rule evaluation and defuzzification
    double numerator = 0;
    double denominator = 0;
    for(int i = 0; i < 5; i++) {
      for(int j = 0; j < 5; j++) {
        double ruleStrength = min(errorMemberships[i], deltaErrorMemberships[j]);
        double outputValue = outputSets[rules[i][j]].b; // Use peak of output set
        numerator += ruleStrength * outputValue;
        denominator += ruleStrength;
      }
    }
    double deltaPWM = (denominator > 0) ? numerator / denominator : 0;

    // Combine fuzzy output with integral term
    deltaPWM += KI * integral;

    // Apply feedforward and fuzzy output
    if (bestPWM > 0 && measuredRPM < 240) { // Only use bestPWM if RPM is reasonable
      output = bestPWM + deltaPWM;
    } else {
      output = 125 + deltaPWM; // Lower default PWM
    }
    
    // Constrain output with safety limits
    output = constrain(output, MIN_PWM, MAX_PWM);
    
    // Apply to motor
    analogWrite(MOTOR_PWM_PIN, (int)output);
    
    // Track optimal PWM when settled
    trackOptimalPWM(now);
    
    // Diagnostic output
    printDiagnostics(count, now);
    
    lastError = error;
    lastSampleTime = now;
  }
}

double getMembership(double x, FuzzySet set) {
  if (x <= set.a || x >= set.c) return 0;
  if (x < set.b) return (x - set.a) / (set.b - set.a);
  return (set.c - x) / (set.c - set.b);
}

void trackOptimalPWM(unsigned long currentTime) {
  // Check if we're settled at target RPM
  if (abs(error) < settleThreshold) {
    if (settleStartTime == 0) {
      settleStartTime = currentTime;
    } 
    else if (currentTime - settleStartTime > settleTime) {
      bestPWM = output; // Record the optimal PWM value
      lastBestPWMReset = currentTime; // Update reset timer
    }
  } 
  else {
    settleStartTime = 0; // Reset if we leave settled state
    // Reset bestPWM if unsettled or RPM too high
    if (currentTime - lastBestPWMReset > BEST_PWM_RESET_INTERVAL || measuredRPM > 240) {
      bestPWM = 0;
      lastBestPWMReset = currentTime;
    }
  }
}

void printDiagnostics(unsigned long count, unsigned long currentTime) {
  Serial.print("Target:"); Serial.print(targetRPM, 0);
  
  Serial.print(" RPM\tActual:"); Serial.print(measuredRPM, 0);
  Serial.print("\tCount:"); Serial.println(count);
}

void encoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastEncoderTime > DEBOUNCE_TIME) {
    encoderCount++;
    lastEncoderTime = currentTime;
  }
}