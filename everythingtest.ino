#include <Wire.h>
#include "Adafruit_MCP9808.h"

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

int fsrPin = 2;     // the FSR and 10K pulldown are connected to a0
double fsrReading;     // the analog reading from the FSR resistor divider
double fsrVoltage; // the analog reading converted to voltage
double pressure;
double pressure2;
double fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
double fsrConductance; 
double fsrForce; 

float tempAnalog = 0.0;
double tempVol = 0.0;
double temperatureCelcius =0.0;

int HIH4030_Pin = A0;

void setup() 

{
  pinMode(A0,INPUT);
  Serial.begin(9600);
   
   }

float getHumidity(float degreesCelsius){


//caculate relative humidity
float supplyVolt = 5.0;
 
// read the value from the sensor:
int HIH4030_Value = analogRead(HIH4030_Pin);
float voltage = HIH4030_Value/1023. * supplyVolt; // convert to voltage value
float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius); //temperature adjustment

return trueRH;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  
}
   
void loop() 

{

  fsrReading = analogRead(fsrPin);  
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  //fsrVoltage = mapfloat(fsrReading, 0.0, 1023.0, 0.0, 5000.0);
  fsrVoltage = (fsrReading*5)/1024.0;
  if (fsrVoltage == 0) 
  
  {
    Serial.println("No pressure");
    delay (1000);
  } 
  
  else 
  
  {
    
    fsrResistance = 5000 - fsrVoltage;    
    fsrResistance *= 10000;               
    fsrResistance /= fsrVoltage;

  
    
    fsrConductance = 1000000;            
    fsrConductance /= fsrResistance;
   
    
    if(fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
        
   } 
   
   else 
   
   {
      fsrForce = fsrConductance - 1000;
      
      fsrForce /= 30;
          
    }
 
  pressure=fsrForce/0.0019088161 ;
  pressure2=pressure*0.00750062;
  Serial.print("Pressure in mm/hg: ");
  Serial.println(pressure2); 
  
  //delay(1000);

  tempAnalog = analogRead(A1);
  tempVol = ((tempAnalog*3.3)/1024.0)*1000;
  temperatureCelcius = (-0.07371*tempVol)+194.32;
  
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;
  Serial.print("MCP9808 temperature : ");
  Serial.print(c); 
  Serial.println("");

  float relativeHumidity = getHumidity(c);
  
  Serial.println(relativeHumidity);

delay(1000);  
} }
