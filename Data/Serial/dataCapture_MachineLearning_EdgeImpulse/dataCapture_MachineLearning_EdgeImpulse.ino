/* 
  This is a program to help with data capture for training a machine learning model. This works with Edge Impulse's software.
  You'll need to use this in conjunction with the Edge Impulse Command Line Data Forwarder
  https://docs.edgeimpulse.com/docs/tools/edge-impulse-cli/cli-data-forwarder
  If you're modifying this sketch, new bits of data should be separated with a comma or a TAB
  you can modifty how long you sample for (SAMPLING_PERIOD_MS) and how often you sample (SAMPLING_FREQ_HZ)
  Process:
  1. Upload a version of this script to your board
  2. Run the data forwarder
  3. plug in your board 
  4. tap the button attached to your board to start a new gesture
   */

#define BTN_PIN 12  // Button pin

#define SAMPLING_FREQ_HZ 100                        // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS 1000 / SAMPLING_FREQ_HZ  // Sampling period (ms)
#define NUM_SAMPLES 100   

// Pins
const int ldrPin = A3; // LDR connected to pin A3
const int ldrPin2 = A2; //LDR connected to pin A2
const int ledPin = D7; // LED connected to pin D7

// Variables for smoothing
float alpha = 0.1;
float smoothedLdrValue = 0;
float smoothedLdrValue2 = 0;]


void setup() {

  pinMode(BTN_PIN, INPUT_PULLUP);// Enable button pin

  pinMode(ldrPin, INPUT); //pin function as input
  pinMode(ldrPin2, INPUT);
  pinMode(ledPin, OUTPUT);//pin function as output

  Serial.begin(115200);//speed at which microcontroller is speaking

}

void loop() {

    unsigned long timestamp;
  unsigned long start_timestamp;

  // Wait for button press
  while (digitalRead(BTN_PIN) == 1)
    ;
  Serial.print("timestamp,");
  for (int i = 0; i < 25; i++) {
    Serial.print("p");
    Serial.print(String(i));
    if(i<25-1){
      Serial.print(",");
    }
    
  }
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

}

void readSensors(){
  int ldrValue = analogRead(ldrPin);
  smoothedLdrValue = alpha * ldrValue + (1 - alpha) * smoothedLdrValue;//smoothing data from LDR1
  Serial.print((int)smoothedLdrValue); //printing smoothed value

  int ldrValue2 = analogRead(ldrPin2);
  smoothedLdrValue2 = alpha * ldrValue2 + (1 - alpha) * smoothedLdrValue2; // smoothing data from LDR2
  Serial.print((int)smoothedLdrValue2);//printing smoothed value

  Serial.print(",");

}