/* FSR testing sketch. 
 
Connect one end of FSR to power, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground 
*/
 
int fsrPin[5] = {A0, A1, A2, A3, A6};     // the FSR and 10K pulldown are connected to a0
int fsrReading[5] = {0};     // the analog reading from the FSR resistor divider
int fsrVoltage[5] = {0};     // the analog reading converted to voltage
unsigned long fsrResistance[5] = {0};  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance[5] = {0}; 
long fsrForce[5] = {0};       // Finally, the resistance converted to force

 
void setup(void) {
  Serial.begin(115200);   // We'll send debugging information via the Serial monitor
}
 
void loop(void) {
  for(int i = 0; i<5; i++) {
    fsrReading[i] = analogRead(fsrPin[i]);  
    Serial.print("SENSOR "); Serial.print(i+1); Serial.print(": Analog reading "); Serial.print(i+1); Serial.print(" = "); Serial.print(fsrReading[i]);
   
    // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
    fsrVoltage[i] = map(fsrReading[i], 0, 1023, 0, 5000);
    //Serial.print("\tVoltage reading in mV = ");
    //Serial.print(fsrVoltage[i]);  
   
    if (fsrVoltage[i] == 0) {
      Serial.println("\tNo pressure");  
    } else {
      // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
      // so FSR = ((Vcc - V) * R) / V        yay math!
      fsrResistance[i] = 5000 - fsrVoltage[i];     // fsrVoltage is in millivolts so 5V = 5000mV
      fsrResistance[i] *= 10000;                // 10K resistor
      fsrResistance[i] /= fsrVoltage[i];
      //Serial.print("\tFSR resistance in ohms = ");
      //Serial.print(fsrResistance[i]);
   
      fsrConductance[i] = 1000000;           // we measure in micromhos so 
      fsrConductance[i] /= fsrResistance[i];
     // Serial.print("\tConductance in microMhos: ");
     // Serial.print(fsrConductance[i]);
   
      // Use the two FSR guide graphs to approximate the force
      if (fsrConductance[i] <= 1000) {
        fsrForce[i] = fsrConductance[i] / 80;
        Serial.print("\tForce in Newtons: ");
        Serial.println(fsrForce[i]);      
      } else {
        fsrForce[i] = fsrConductance[i] - 1000;
        fsrForce[i] /= 30;
        Serial.print("\tForce in Newtons: ");
        Serial.println(fsrForce[i]);            
      }
    }
  }
    Serial.println("--------------------");
    delay(500);
}
