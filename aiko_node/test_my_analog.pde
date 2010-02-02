/*
**  A very simple thing for geting the analog data from the 'drino
*/
#define S1_PIN 3    // pin for the analog data input

#define SAMPLES 300 // number of samples to take

int val1 = 0;

int reading_cnt = 0; // counter for 1 second samples
float total_rms1 = 0;


void setup() {
//  analogReference(INTERNAL);
  Serial.begin(38400);
}

void loop() {
  
  int max_val1 = 0;
  float rms_val1 = 0;
  
 
   for (int cnt=0; cnt < SAMPLES; cnt++) {
    val1 = analogRead(S1_PIN);
    Serial.println(val1);
    rms_val1 = rms_val1 + sq((float)val1);
  }
  
  rms_val1 = sqrt(rms_val1 / (SAMPLES/2) );
  total_rms1 = total_rms1 + rms_val1;


  reading_cnt++;
  
  //if (reading_cnt >= NUM_READINGS) {

  // 14.8 is a special number - derived from observed power (current * 240) / average_rms1
    float average_rms1 = total_rms1 * 14.8;
        
    Serial.print("(power ");
    Serial.print(average_rms1);
    Serial.println(" W)");
    
    total_rms1 = 0;
    reading_cnt = 0;
  //}
  
  delay(20000);

}
