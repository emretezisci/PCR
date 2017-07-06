
 /*Project Name: Thermal Controller
 
 Definition: The program reads the potentiometers value and depending of the value, the brightness
 of the halogen lamp is controlled. The value on the thermistor pin is read and the resistance and the
 temperature is caluclated using the Steinhart-Hart equation and depending on the temperature
 the fan is turned on and off. The speed of the fan is adjustable.*/

#define PotPin 6             //P1.4  ----  This pin is used to read the value of the potentiometer
#define ThermistorPin 7      //P1.5  ----  This pin is used to read the value of the termistor
#define HalogenLampPin 14    //P1.6  ----  This pin is used to write the value from the potentiometer
#define FanPowerPin 12       //P2.4  ----  This pin is used to turn the fan on or off
#define FanSpeedPin 15       //P1.7  ----  This pin is used to control the speed of the fan
#define LEDPin 10            //P2.2  ----  This pin is used to write to the LED

int PotValue;               //Variable to hold the potentiometer value
double ThermistorValue;        //Variable to hold the thermistor value
double Resistance;           //Variable to hold the resistance value of the thermistor
double Temperature;          //Variable to hold the temperature value of the thermistor
double Temperature0;         // Temp000
int InternalSensor;         //Variable to hold the value of the internal temperature sensor
int InternalTemperature;    //Variable to hold the temperature value of the internal temperature sensor
double FloatTemperature;     //Variable to hold the float value of the internal temperature sensor
int AlertCount;             //Variable to hold the count of the alerts where Temperature > Internal Temperature + 10 *C

void setup()  {
  Serial.begin(9600);                              //Start data transmission
  pinMode(HalogenLampPin, OUTPUT);                 //Set HalogenLampPin as output
  //pinMode(FanPowerPin, OUTPUT);                    //Set FanPowerPin as output
  pinMode(LEDPin, OUTPUT);                         //Set LEDPin as output
  AlertCount = 0;                                  //Initial count of the alert is 0
  Serial.println("***Thermal Controller***\n");    //Print a welcome message
  delay(3000);                                     //A delay before showing the data
} 

void loop()  {
  HalogenLamp();        //Initialize the Halogen Lamp function
  Thermistor();         //Initialize the Thermistor function
  CoreTemperature();    //Initialize the Core Temperature function
  //Fan();              //Initialize the Fan function

  unsigned long time = millis();                            //Time passed from the beginning
  const unsigned long halfMinutes = 0.5 * 60 * 1000;        //Half minute of time
  static unsigned long lastSampleTime = 0 - halfMinutes;    //Last sample time

  if ((time - lastSampleTime) >= halfMinutes){          //If Half minute is passed, do the desired operations
    lastSampleTime = lastSampleTime + halfMinutes;    //Set the time of sample
    if(Temperature >= (FloatTemperature - 10)){         //If Temperature is greater than core temperature + 10 *C
      AlertCount = AlertCount + 1;                    //Increase the count of alert
      Serial.print("Count is: ");                     //Display count to the Serial Monitor
      Serial.print(AlertCount);                       //Display the count as a decimal number
      Serial.print("\t");
      PrintBits(AlertCount);                          //Display the count as an 8 bit binary number
      Serial.println("\n");
      BlinkBits(AlertCount);                          //Dislay binary number as LED blinks, 8 bits LSB to MSB
    }
  }
}

/*Controlling the Halogen Lamp
 *The potentiometers value is read and scaled
 *and then the value is written to the halogen lamp
 *to adjust its brightness
 */
 
void HalogenLamp(void){
  PotValue = analogRead(PotPin) * (255.0 / 1023);    //Read the value from the potentiometer and scale the value
  analogWrite(HalogenLampPin, PotValue);             //Write the scaled value to HalogenLampPin
  Serial.print("Potentiometer value is: ");          //Print the value to the Serial Monitor
  Serial.println(PotValue);
  delay(10);                                         //Delay between the values
}

/*Steinhart-Hart equation to calculate the temperature from resistance
 *Resistance = (1024 * Resistance / ADC) - Resistor
 *Temperature in Kelvin = 1 / {A + B*[ln(R)] + C*[ln(R)]^3}
 *Resistor = 10 KOhm = 10000
 *A = 0.001129148, B = 0.000234125 and C = 0.0000000876741
 */
 
void Thermistor(void) {  
  
  
  ThermistorValue = analogRead(ThermistorPin) / 1000.0;                                                                                    //Read the value of the ThermistorPin
  Resistance = ((10230.0 / ThermistorValue) - 10000.0) / 9250.0;                                                                              //Calculate resistance from the read value using the resistance equation 
  Serial.println(ThermistorValue);
  Temperature0 = logf(Resistance) / 3977.0;                                                                                                 //Take the natural logarithm of the resistance  
  
  //Temperature = 1 / (0.001129148 + (0.000234125 * Temperature) + (0.0000000876741 * Temperature * Temperature * Temperature));    //Apply the temperature equation
  Temperature = 1.0 / (1/298.15 + Temperature0);
  Temperature = Temperature - 273.15;                                                                                             //Convert Kelvin to Celsius
  Serial.print("Temperature is: ");                                                                                               //Print the values as Celsius to the Serial Monitor
  Serial.print(Temperature);
  Serial.println(" *C");
  delay(100);                                                                                                                     //Delay between the values
}

/*Calculating the core temperature of MSP430
 *From MSP430 User guide page 571:
 *Voltage = 0.00355 * Celcius + 0.986
 *0.00355 * Celcius + 0.986 = Voltage
 *Celcius = Voltage / 0.00355 - 277.75
 *For 1.5V reference: Voltage = ADC * 1.5 / 1023 = ADC * 0.0014663
 *Celcius = ADC * 0.0014663 / 0.00355 - 277.75 = ADC * 0.413 - 277.75
 *Celcius * 65536 (2^16) / 65536 = (ADC * 27069 - 18202393) / 65536 =  (ADC * 27069 - 18202393) >> 16
 *Add 0.5 * 65536 to improve rounding
 *Celcius = (ADC * 27069 - 18169625) >> 16
 */
 
void CoreTemperature(void){
  analogReference(INTERNAL1V5);                                                                  //Use 1.5V reference
  InternalSensor= analogRead(TEMPSENSOR);                                                        //Read the value of the internal temperature sensor
  InternalTemperature = ((uint32_t)InternalSensor * 27069 - 18169625) * 10 >> 16;                //Apply the temperature equation
  FloatTemperature = (float)((InternalTemperature / 10)) + (InternalTemperature % 10) / 10.0;    //Get the float value of the temperature
  Serial.print("Internal temperature is: ");                                                     //Print the values as Celsius to the Serial Monitor
  Serial.print(FloatTemperature);
  Serial.println(" *C");
  delay(100);                                                                                    //Delay between the values
}

/*Controlling the Fan
 *The Temperature is checked and if it is greater
 *than internal temperature + 10 *C the fan is turned on
 *otherwise, the fan remains off
 */
 
void Fan(void){
  if(Temperature > FloatTemperature + 10)
    digitalWrite(FanPowerPin, HIGH);
  else
    digitalWrite(FanPowerPin, LOW);
}

/*Blinking the LED with respect to the value of an 8 bit binary number LSB to MSB*/

void BlinkBits(byte myByte){                              
  for(int i = 0; i < 8; i++){        //Checking all 8 bits
    if(bitRead(myByte, i) == 1){     //If the value of a bit is 1 
      digitalWrite(LEDPin, HIGH);    //Turn on the LED
      delay(1000);                   //For 1 seconds
      digitalWrite(LEDPin, LOW);     //Turn off the LED 
      delay(250);                    //For 250 milliseconds, used for identification between consecutive 1's 
    }
    else{                            //If the value of a bit is 0
      digitalWrite(LEDPin, LOW);     //Turn off the LED
      delay(1000);                   //For 1 seconds
    }
  }
}

/*Printing a decimal number as 8 bit binary*/

void PrintBits(byte myByte){
  for(byte mask = 0x80; mask; mask >>= 1){    //Check the number by shifting 10000000 to right
    if(mask & myByte)                         //If the value of a bit is 1
      Serial.print('1');                      //Print 1
    else                                      //If the value of a bit is 0
      Serial.print('0');                      //Print 0
  }
}