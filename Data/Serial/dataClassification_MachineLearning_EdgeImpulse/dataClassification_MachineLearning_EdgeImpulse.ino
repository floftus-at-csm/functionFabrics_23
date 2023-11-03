#define DELAY_COLLECTION 1000                       // Delay (ms) between collections
#define SAMPLING_FREQ_HZ 100                        // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS 1000 / SAMPLING_FREQ_HZ  // Sampling period (ms)
#define NUM_SAMPLES 100                             // 100 samples at 100 Hz is 1 sec window

// Pins
const int ldrPin = A3;   // LDR connected to pin A3
const int ldrPin2 = A2;  //LDR connected to pin A2
const int ledPin = D7;   // LED connected to pin D7

// Variables for smoothing
float alpha = 0.1;
float smoothedLdrValue = 0;
float smoothedLdrValue2 = 0;

void setup() {

  pinMode(BTN_PIN, INPUT_PULLUP);  // Enable button pin

  pinMode(ldrPin, INPUT);  //pin function as input
  pinMode(ldrPin2, INPUT);
  pinMode(ledPin, OUTPUT);  //pin function as output

  Serial.begin(115200);  //speed at which microcontroller is speaking
}

void loop() {

  unsigned long timestamp;
  unsigned long start_timestamp;

  // Print header
  Serial.print("timestamp,");
  Serial.print('ldr1');
  Serial.print(',ldr1');
  Serial.println();


  start_timestamp = millis();
  for (int i = 0; i < NUM_SAMPLES; i++) {

    // Take timestamp so we can hit our target frequency
    timestamp = millis();
    Serial.print(timestamp - start_timestamp);

    Serial.print(",");

    readSensors();
  }
  Serial.println();
  // Wait before repeating the collection process
  delay(DELAY_COLLECTION);
}

void readSensors() {
  int ldrValue = analogRead(ldrPin);
  smoothedLdrValue = alpha * ldrValue + (1 - alpha) * smoothedLdrValue;  //smoothing data from LDR1
  Serial.print((int)smoothedLdrValue);                                   //printing smoothed value

  int ldrValue2 = analogRead(ldrPin2);
  smoothedLdrValue2 = alpha * ldrValue2 + (1 - alpha) * smoothedLdrValue2;  // smoothing data from LDR2
  Serial.print((int)smoothedLdrValue2);                                     //printing smoothed value

  Serial.print(",");
}