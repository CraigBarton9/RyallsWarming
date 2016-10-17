// heater driver
// arduino Nano reading MLX90614 IR thermometer over i2C
// outputing temperature on D5 using PWM (0-5V = 0-50C) 
// reading setpoint on A7 (0-5V = 0-50C) 
// using PID control on D6 to drive SSR and 1KW heater
// display on OLED i2C display
// Craig Barton modified from a number of sources.
#include <SimpleModbusSlave.h>
#include <i2cmaster.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <avr/wdt.h>

#define OLED_RESET 4
#define RELAY_PIN 6 //digital out
#define POT_Pin 7 //analog input
#define TempOUT_Pin 5 //pwm digital out

Adafruit_SSD1306 display(OLED_RESET);


const int chipSelect = 10;
#define XPOS 0
#define YPOS 1
#define DELTAY 2

//////////===============================
int modbusaddress=12;
//========================================
//Define Variables we'll be connecting to via Processing
double Setpoint, Input, Output;

// Using the enum instruction allows for an easy method for adding and 
// removing registers. Doing it this way saves you #defining the size 
// of your slaves register array each time you want to add more registers
// and at a glimpse informs you of your slaves register layout.

//////////////// registers of your slave ///////////////////
enum 
{     
  // just add or remove registers and your good to go...
  // The first register starts at address 0
  BODY_TEMP,     
  TARG_TEMP,        
  SETPNT,
  DUTY_CYCLE,
  TOTAL_ERRORS,
  // leave this one
  TOTAL_REGS_SIZE 
  // total number of registers for function 3 and 16 share the same register array
};

unsigned int holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////////////////////////////////////


//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
unsigned long serialTime; //this will help us know when to talk with processing
int WindowSize = 1000;
unsigned long windowStartTime;

// check OLED display
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

float targT;
float bodyT;

void setup(){
//	Serial.begin(9600);
//	Serial.println("Setup...");
// immediately disable watchdog timer so set will not get interrupted

    wdt_disable(); 
//---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
 
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (The DEFAULT)
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
 
 
//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
 
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
 
//---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------
 
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
 
//== MODBUS stuff===
  /* parameters(long baudrate, 
                unsigned char ID, 
                unsigned char transmit enable pin, 
                unsigned int holding registers size)
                
     The transmit enable pin is used in half duplex communication to activate a MAX485 or similar
     to deactivate this mode use any value < 2 because 0 & 1 is reserved for Rx & Tx
  */
  
  modbus_configure(&Serial,9600,SERIAL_8N1, modbusaddress, 2, TOTAL_REGS_SIZE,holdingRegs);
  modbus_update_comms(9600, SERIAL_8N2, modbusaddress);
  wdt_enable(WDTO_8S); //watchdog timer set to 8s
//=================  
//  analogReference(DEFAULT);
  pinMode(RELAY_PIN, OUTPUT);      // sets the digital pin as output
  digitalWrite(RELAY_PIN, LOW);
  windowStartTime = millis();
	Input = 15;
  Setpoint = 10;
    //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
    //turn the PID on
  myPID.SetMode(AUTOMATIC);
	i2c_init(); //Initialise the i2c bus used to talk to temp sensor and OLED display
	PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  // Clear the buffer.
  display.clearDisplay();
}

void loop(){
    
    wdt_reset(); //reset watchdog timer
    int dev = 0x5A<<1;
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    holdingRegs[TOTAL_ERRORS] = modbus_update();
    Setpoint = double(holdingRegs[SETPNT])/100.0;
   //may need to add error checking here
   
  //read the IR temperature sensor data over I2C both target and then sensor body
    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x07); //address for temperature of target
    // read
    i2c_rep_start(dev+I2C_READ);
    data_low = i2c_readAck(); //Read 1 byte and then send ack
    data_high = i2c_readAck(); //Read 1 byte and then send ack
    pec = i2c_readNak();
    i2c_stop();
    
    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    double tempData = 0x0000; // zero out the data
    int frac; // data past the decimal point
    
    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    tempData = (tempData * tempFactor)-0.01;
    
    targT = tempData - 273.15;
    
    // read bodyTa from device
    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x06); //address for temperature of sensor body
    
    // read
    i2c_rep_start(dev+I2C_READ);
    data_low = i2c_readAck(); //Read 1 byte and then send ack
    data_high = i2c_readAck(); //Read 1 byte and then send ack
    pec = i2c_readNak();
    i2c_stop();
    
    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
//    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    tempData = 0x0000; // zero out the data
 //   int frac; // data past the decimal point
    
    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    tempData = (tempData * tempFactor)-0.01;
    
   bodyT = tempData - 273.15;   
   

 //place data on holding registers for comunication to logger
 //convert from float to int after moving 2 dec places.
   holdingRegs[TARG_TEMP]=int(targT*100.0);
   holdingRegs[BODY_TEMP]=int(bodyT*100.0);
   holdingRegs[DUTY_CYCLE]=int(Output);
     
 //display data on the OLED display
   display.clearDisplay();
   display.setTextSize(2);
   display.setTextColor(WHITE);
 //  display.setCursor(0,50);
 //  display.print("Duty :");
 //  display.print((Output/WindowSize)*100,1);   
   display.setCursor(0,0);
   display.print("BodyT:");
   display.print(bodyT,1);
   display.setCursor(0,17);
   display.print("StPnt:");
   display.print(Setpoint,1);  
   display.setCursor(0,35); 
   display.print("TTemp:");
   display.print(targT,1);
   display.setCursor(0,55);
   display.setTextSize(1);
   display.print("addrs ");
   display.print(modbusaddress);
   display.setCursor(60,55); 
   display.print("duty:"); 
   display.print(Output);  
   display.display();
  // rebuild the picture after some delay
  //  delay(500); // wait a second before printing again

  //pid stuff
  Input=targT;
  myPID.Compute();
   /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output > millis() - windowStartTime) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);


   
}




