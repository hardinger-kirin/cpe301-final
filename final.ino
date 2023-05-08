/*
  Kirin Hardinger
  CPE 301 Final Project
  Spring 2023
*/

// States
enum STATE_E {
  IDLE = 0,
  RUNNING = 1,
  DISABLED = 2,
  ERROR = 3
};

STATE_E current_state = DISABLED;
STATE_E last_state = current_state;

// Bit manipulation functions
#define BIT_SET(x, y) (x) |= (1 << (y))
#define BIT_CLEAR(x, y) (x) &= ~(1 << (y))
#define BIT_TOGGLE(x, y) (x) ^= (1 << (y))
#define BIT_READ(x, y) ((x) & (1 << (y)))

// RTC
#include <DS3231.h>
DS3231 clock;
RTCDateTime dt;

// Servo
#include <Servo.h>
Servo myservo;

// Temp and humidity sensor
#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11
#define DHT_SENSOR_PIN 6
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
#define THRESHOLD_TEMP 25
float temp, humidity;

// LCD display
#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// ---UART---
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  =  (unsigned int *)0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// ****************PORTS****************
// ---ADCs---
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// ---LEDs---
volatile unsigned char* port_c = (unsigned char*) 0x28;
volatile unsigned char* ddr_c = (unsigned char*) 0x27;
volatile unsigned char* pin_c = (unsigned char*) 0x26;
#define LED_PORT *port_c
#define RED_PIN 4
#define YELLOW_PIN 5
#define GREEN_PIN 6
#define BLUE_PIN 7

// ---On-Off Button---
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;
volatile unsigned char* myPCICR = (unsigned char*) 0x68;
volatile unsigned char* myPCMSK0 = (unsigned char*) 0x6B;
#define BUTTON_PIN 7

// ---Fan Motor---
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C;

void setup() {
  // UART setup
  U0init(9600);

  // RTC
  clock.begin();
  clock.setDateTime(__DATE__, __TIME__);

  // Servo
  myservo.attach(2);

  // ADC for potentiometer to control Servo and water sensor
  adc_init();

  // LCD display
  lcd.begin(16, 2);

  // ---Interrupt Setup---
  // Enable PCIE0 which is for PCINT7:0 pins (button is on PCINT7)
  BIT_SET(*myPCICR, 0);
  // Enable PCMSK0 on bit 7 which is for PCINT7
  BIT_SET(*myPCMSK0, 7);

  // ---LEDs to OUTPUT---
  BIT_SET(*ddr_c, RED_PIN); // RED is on PC4
  BIT_SET(*ddr_c, YELLOW_PIN); // YELLOW is on PC5
  BIT_SET(*ddr_c, GREEN_PIN); // GREEN is on PC6
  BIT_SET(*ddr_c, BLUE_PIN); // BLUE is on PC7

  // On-Off button to INPUT
  BIT_CLEAR(*ddr_b, BUTTON_PIN); // Button is on PB7

  // Fan motor to OUTPUT
  BIT_SET(*ddr_e, 3); // Motor is on PE3
}

void loop() {
  // check for water level - if it's too low, go to ERROR state
  if(read_water_level() < 950) {
    current_state = ERROR;
  }

  if(current_state != ERROR and current_state != IDLE) {
    // track potentiometer movement to update servo angle
    myservo.write(adc_read(11) / 4); // potentiometer is on A11, divide by 4 to scale angle of servo

    // display temp and humidity to LCD
    display_temp_humidity();
    measure_environment(&temp, &humidity);
  }

  switch(current_state) {
    case IDLE:
      reset_leds();
      BIT_SET(LED_PORT, GREEN_PIN); // turn on green LED

      // check for temp - if it's over threshold, go to RUNNING state
      if(temp > THRESHOLD_TEMP) {
        current_state = RUNNING;
        set_motor(true);
        display_time();
      }
      break;
    case RUNNING:
      reset_leds();
      BIT_SET(LED_PORT, BLUE_PIN); // turn on blue LED

      // check for water level - if it's too low, go to ERROR state
      if(read_water_level() < 950) {
        current_state = ERROR;
      }

      // check for temp - if it's below threshold, go to IDLE state
      if(temp < THRESHOLD_TEMP) {
        current_state = IDLE;
        set_motor(false);
        display_time();
      }
      break;
    case DISABLED:
      set_motor(false); // turn off fan
      reset_leds();
      BIT_SET(LED_PORT, YELLOW_PIN); // turn on yellow LED
      break;
    case ERROR:
      set_motor(false); // turn off fan
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("!!Error!!"); // display error message
      reset_leds();
      BIT_SET(LED_PORT, RED_PIN); // turn on red LED
      break;
  }
}

int read_water_level() {
  return adc_read(15); // water sensor is on A15
}

void set_motor(bool enable) {
  if(enable) {
    BIT_SET(*port_e, 3);
  } else {
    BIT_CLEAR(*port_e, 3);
  }
}

bool measure_environment(float* temp, float* humidity) {
  if(dht_sensor.measure(temp, humidity)) {
    return(true);
  }

  return(false);
}

void display_temp_humidity() {
  char printBuffer[128];

  lcd.setCursor(0, 0);
  sprintf(printBuffer, "Temperature: %dC", int(temp));
  lcd.print(printBuffer);

  lcd.setCursor(0, 1);
  sprintf(printBuffer, "Humidity: %d%%", int(humidity));
  lcd.print(printBuffer);
}

void display_time() {
  dt = clock.getDateTime();
  char str[50];

  // sprintf returns number of characters added to the array - use this to loop over the string and send each char to Serial Monitor
  int num_chars = sprintf(str, "%d-%d-%d %d:%d:%d", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);

  for(int i = 0; i < num_chars; i++) {
    U0putchar(str[i]);
  }

  U0putchar('\n');  
}

void reset_leds() {
  BIT_CLEAR(LED_PORT, RED_PIN);
  BIT_CLEAR(LED_PORT, YELLOW_PIN);
  BIT_CLEAR(LED_PORT, GREEN_PIN);
  BIT_CLEAR(LED_PORT, BLUE_PIN);
}

void adc_init() {
  BIT_SET(*my_ADCSRA, 8);
  BIT_SET(*my_ADCSRB, 6);
  BIT_SET(*my_ADMUX, 7);
}

unsigned int adc_read(unsigned char adc_channel_num) {
  *my_ADMUX  &= 0xE0;
  BIT_CLEAR(*my_ADCSRB, 4);

  // set the channel number
  if(adc_channel_num > 7) {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0x8;
  }
  *my_ADMUX  += adc_channel_num;
  
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

void U0init(unsigned long U0baud) {
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = (16000000 / 16 / U0baud - 1);
}

void U0putchar(unsigned char U0pdata) {
  //wait until bit 5 is set - meaning data register is empty and ready to receive new data
  while(!(*myUCSR0A & (1<<5))){};
  *myUDR0 = U0pdata;
}

ISR(PCINT0_vect) {
  if(BIT_READ(*pin_b, BUTTON_PIN)) {
    switch(current_state) {
      case IDLE:
        current_state = DISABLED;
        break;
      case RUNNING:
        current_state = DISABLED;
        break;
      case DISABLED:
        current_state = IDLE;
        break;
      case ERROR:
        current_state = IDLE;
        break;
    }
  }
}