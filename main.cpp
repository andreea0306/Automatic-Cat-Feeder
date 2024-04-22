// used for HX711
#include "hx711/include/common.h"
#include <cmath>
#include <cstdlib>
#include <string>
#include <stdlib.h>
#include <wiringPiI2C.h>
#include "Timer.cpp"

using namespace HX711;

// global variables
int n1, n2;
double readValue;
char array1[] = "Food quantity is";
char array2[] = "Bowl is already full";
// Define some device parameters
#define I2C_ADDR   0x27 // I2C device address

// Define some device constants
#define LCD_CHR  1 // Mode - Sending data
#define LCD_CMD  0 // Mode - Sending command

#define LINE1  0x80 // 1st line
#define LINE2  0xC0 // 2nd line

#define LCD_BACKLIGHT   0x08  // On
// LCD_BACKLIGHT = 0x00  # Off

#define ENABLE  0b00000100 // Enable bit

void lcd_init(void);
void lcd_byte(int bits, int mode);
void lcd_toggle_enable(int bits);
// added by Lewis
void typeInt(int i);
void typeFloat(float myFloat);
void typeDouble(double myDouble);
void lcdLoc(int line); //move cursor
void ClrLcd(void); // clr LCD return home
void typeln(const char *s);
void typeChar(char val);

int fd;  // seen by all subroutines

void moveServo() {

    std::cout<<"[DEBUG]: first move"<<std::endl;
    pwmWrite(piPin, 550);
    delay(500);
    std::cout<<"[DEBUG]: second move back"<<std::endl;
    pwmWrite(piPin, 350);
    delay(500);

}

void timerCallback() {
    //hxGlobal.setUnit(Mass::Unit::G);
    std::cout << "[DEBUG]: Timer ISR called!"<<std::endl;
    if(readValue <= 30.00) {
        ClrLcd();
	lcdLoc(LINE1);
        typeln(array1);
	lcdLoc(LINE2);
	typeDouble(readValue);
	typeln(" grams");
        moveServo();
    }
    else {
	ClrLcd();
	lcdLoc(LINE1);
	typeln(array2);
	lcdLoc(LINE2);
	typeDouble(readValue);
	typeln(" grams");
        std::cout<<"[DEBUG]: THE BOWL IS FULL ENOUGH"<<std::endl;
    }
    //std::cout<<"[DEBUG]: n1:"<<n1<<" and n2:"<<n2<<std::endl;
    std::cout << "[DEBUG]: read value is:" << readValue << std::endl;
}

void init() {
    wiringPiSetup();
    fd = wiringPiI2CSetup(I2C_ADDR);
    lcd_init(); // setup LCD
    pinMode(piPin, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(192);
    pwmSetRange(2000);
// set sservo to the 0 point
    pwmWrite(piPin, 350);
}

int main(int argc, char *argv[]) {
    
    // Create a timer with a 5 seconds interval and an ISR function
    std::cout<<"[DEBUG]: start application"<<std::endl;
    std::cout<<"[DEBUG]: init wires"<<std::endl;
    init();
    lcdLoc(LINE1);
    typeln(array1);
    n1 =  atoi(argv[1]);
    n2 = atoi(argv[2]);
    SimpleHX711 hx(23, 24, n1, n2);
    hx.setUnit(Mass::Unit::G);
    //std::cout<<"[DEBUG]: value from sensor is:"<<hx.weight(15)<<std::endl;
    std::cout<<"[DEBUG]: start timer"<<std::endl;
    Timer timer(std::chrono::milliseconds(5000), timerCallback);

    // Start the timer
    timer.start();
    for(;;){
    // Run this forever
      readValue = hx.weight(15);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    // Stop the timer
    timer.stop();

    return 0;

}

// float to string
void typeFloat(float myFloat)   {
  char buffer[20];
  sprintf(buffer, "%4.2f",  myFloat);
  typeln(buffer);
}

// int to string
void typeInt(int i)   {
  char array1[20];
  sprintf(array1, "%d",  i);
  typeln(array1);
}

void typeDouble(double myDouble) {
  char buffer[20];
  sprintf(buffer, "%.3lf", myDouble);
  typeln(buffer);
}

// clr lcd go home loc 0x80
void ClrLcd(void)   {
  lcd_byte(0x01, LCD_CMD);
  lcd_byte(0x02, LCD_CMD);
}

// go to location on LCD
void lcdLoc(int line)   {
  lcd_byte(line, LCD_CMD);
}

// out char to LCD at current position
void typeChar(char val)   {

  lcd_byte(val, LCD_CHR);
}


// this allows use of any size string
void typeln(const char *s)   {

  while ( *s ) lcd_byte(*(s++), LCD_CHR);

}

void lcd_byte(int bits, int mode)   {

  //Send byte to data pins
  // bits = the data
  // mode = 1 for data, 0 for command
  int bits_high;
  int bits_low;
  // uses the two half byte writes to LCD
  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT ;
  bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT ;

  // High bits
  wiringPiI2CReadReg8(fd, bits_high);
  lcd_toggle_enable(bits_high);

  // Low bits
  wiringPiI2CReadReg8(fd, bits_low);
  lcd_toggle_enable(bits_low);
}

void lcd_toggle_enable(int bits)   {
  // Toggle enable pin on LCD display
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits | ENABLE));
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits & ~ENABLE));
  delayMicroseconds(500);
}


void lcd_init()   {
  // Initialise display
  lcd_byte(0x33, LCD_CMD); // Initialise
  lcd_byte(0x32, LCD_CMD); // Initialise
  lcd_byte(0x06, LCD_CMD); // Cursor move direction
  lcd_byte(0x0C, LCD_CMD); // 0x0F On, Blink Off
  lcd_byte(0x28, LCD_CMD); // Data length, number of lines, font size
  lcd_byte(0x01, LCD_CMD); // Clear display
  delayMicroseconds(500);
}
