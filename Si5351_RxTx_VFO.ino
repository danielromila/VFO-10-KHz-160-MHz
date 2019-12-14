/*
  I (Daniel Romila) simplified this program in order to use it as a simple, single OUTPUT VFO
  from 10 KHz to 160 MHz. Only the first generator output is active, I switched OFF the other two.

  It is necessary to declare, in the old style, the OLED display as 128 x 64, in the Adafruit_SSD1306.h, from the 
  Adafruit_SSD1306 library. Just take out the comments (the //) from in front of: #define SSD1306_128_64
  
  A part of this program is taken from Jason Mildrum, NT7S.
  All extra functions are written by Rob Engberts PA0RWE

  References:
  http://nt7s.com/
  http://sq9nje.pl/
  http://ak2b.blogspot.com/
  http://pa0rwe.nl/?page_id=804

 *  SI5351_VFO control program for Arduino NANO
 *  Copyright PA0RWE Rob Engberts
 *  
 *  Using the old Si5351 library by Jason Mildrun nt7s
 *
 *  Functions:
 *  - CLK0 - Tx frequency = Display frequency
 *  - CLK1 - Rx / RIT frequency = Tx +/- BFO (upper- or lower mixing)
 *           When RIT active, RIT frequency is displayed and is tunable.
 *           When RIT is inactive Rx = Tx +/- BFO
 *  - CLK2 - BFO frequency, tunable
 *
 *  - Stepsize:  select (pushbutton)
 *  - Calibrate: (pushbutton) calculates difference between X-tal and measured
 *               x-tal frequency, to correct x-tal frequency.
 *  - Selection: (pushbutton) Switch between TRx and BFO mode
 *  - RIT switch: tunable Rx frequency, while Tx frequency not changed
 *  - Programming PIC by ICSP
 *  
 *  Si5351 settings: I2C address is in the .h file
 *                   X-tal freq is in the .h file but set in line 354
 *
***************************************************************************
 *  02-04-2015   1.0    Start building program based on the PIC version
 *  18-06-2019   1.1    Extend frequency range to 10 KHz down
 *                      Update frequency display. Added 'MHz' and 'KHz'
 *                      Set max frequency to 100 MHz.
 *
***************************************************************************
*  Includes
**************************************************************************/

#include <Rotary.h>
#include <RWE_si5351.h> 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

/**************************************************************************
*  (Pin) Definitions
**************************************************************************/
#define ENCODER_A     2       // Encoder pin A INT0/PCINT18 D2
#define ENCODER_B     3       // Encoder pin B INT1/PCINT19 D3
#define ENCODER_BTN   4       // Encoder pushbutton D4
#define OLED_RESET    8       // OLED reset
#define Calibrbtn     5       // Calibrate
#define RIT_Switch    6       // RIT Switch
#define TX_Switch     7       // Select TRx or BFO
//      I2C-SDA       A4      // I2C-SDA
//      I2C-SCL       A5      // I2C-SCL

#define F_MIN            10000UL      // Lower frequency limit  10 KHz
#define F_MAX        160000000UL      // Upper frequency limit 200 MHz

/**************************************************************************
*  EEPROM data locations 
**************************************************************************/
#define EE_SAVED_RADIX  0   // Stepsize pointer
#define EE_SAVED_AFREQ  4   // Actual Tx Frequency (CLK0)
#define EE_SAVED_BFREQ  8   // BFO (IF) Frequency  (CLK2)
#define EE_SAVED_XFREQ  12  // X-tal frequency  (25 or 27 MHz)
#define EE_SAVED_OFSET  16  // store correction
#define EE_SAVED_CALBR  20  // calibrated indicator

Adafruit_SSD1306 display(OLED_RESET);
Si5351 si5351;
Rotary r = Rotary(ENCODER_A, ENCODER_B);

/**************************************************************************
*  Declarations
**************************************************************************/
volatile uint32_t bfo_f =  900000000ULL / SI5351_FREQ_MULT;   // CLK0 start IF
volatile uint32_t vfo_t = 1420000000ULL / SI5351_FREQ_MULT;   // CLK2 start Tx freq
volatile uint32_t vfo_r = vfo_t - bfo_f;                      // CLK1 start Rx freq
volatile uint32_t vfo_s = vfo_t;                              // Saved for RIT
uint32_t vco_c = 0;                                           // X-tal correction factor
uint32_t xt_freq;
long radix = 100L, old_radix = 100L;                          //start step size
boolean changed_f = 0, stepflag = 0, calflag = 0, modeflag = 0, ritset = 0;
boolean calibrate = 0;
byte  act_clk = 0, disp_txt = 0;

/**************************************/
/* Interrupt service routine for      */
/* encoder frequency change           */
/**************************************/
ISR(PCINT2_vect) {
  char result = r.process();
  if (result == DIR_CW)
    set_frequency(1);
  else if (result == DIR_CCW)
    set_frequency(-1);
}


/**************************************/
/* Change the frequency               */
/* dir = 1    Increment               */
/* dir = -1   Decrement               */
/**************************************/
void set_frequency(short dir)
{
  switch (act_clk)
  {
    case 0:                 // Tx frequency
      if (dir == 1)
        vfo_t += radix;
      if (dir == -1) {
        if (vfo_t < radix) break; // to prevent negative value
        vfo_t -= radix;
      }
      break;
    case 1:                 // Tx frequency (only if RIT is on)
      if (dir == 1)
        vfo_t += radix;
      if (dir == -1) {
        if (vfo_t < radix) break; // to prevent negative value      
        vfo_t -= radix;
      }
      break;
    case 2:                 // BFO frequency
      if (dir == 1)
        bfo_f += radix;
      if (dir == -1) {
        if (bfo_f < radix) break; // to prevent negative value      
        bfo_f -= radix;
      }
      break;
  }

  if(vfo_t > F_MAX)
    vfo_t = F_MAX;
  if(vfo_t < F_MIN)
    vfo_t = F_MIN;

  changed_f = 1;
}


/**************************************/
/* Read the buttons with debouncing   */
/**************************************/
boolean get_button()
{
  if (!digitalRead(ENCODER_BTN))            // Stepsize
  {
    delay(20);
    if (!digitalRead(ENCODER_BTN))
    {
      while (!digitalRead(ENCODER_BTN));
      stepflag = 1;      
    }
  }
  else if (!digitalRead(Calibrbtn))         // Calibrate
  {
    delay(20);
    if (!digitalRead(Calibrbtn))
    {
      while (!digitalRead(Calibrbtn));
      calflag = 1;
    }
  }
  else if (!digitalRead(TX_Switch))         // Selection
  {
    delay(20);
    if (!digitalRead(TX_Switch))
    {
      while (!digitalRead(TX_Switch));
      modeflag = 1;
    }
  }  
  if (stepflag | calflag | modeflag) return 1;
  else return 0;
}



/**************************************/
/* Displays the frequency and stepsize*/
/**************************************/
void display_frequency()
{
  char LCDstr[10];
  char Hertz[7];
  int p,q = 0;
  unsigned long freq;
  display.clearDisplay();

  switch(act_clk)
  {
    case 0:                               // Tx frequency
      freq = vfo_t;
      break;
    case 1:                               // Tx frequency (Used in RIT Mode)
      freq = vfo_t;
      break;
    case 2:                               // MF frequency
      freq = bfo_f;
      break;
  }
    
  Hertz[1]='\0';                           // empty array

  sprintf(LCDstr, "%ld", freq);           // convert freq to string
  p=strlen(LCDstr);                       // determine length
  display.setCursor(80,20);
  display.setTextSize(2);  
  if (p>6){                               // MHz
    display.print(F("MHz"));
    q=p-6;
    strcpy(Hertz,LCDstr);                 // get Herz digits (6)
    strcpy(LCDstr+q,Hertz+(q-1));         // copy into LCDstr and add to MHz
    LCDstr[q]='.';                        // decimal point
  }
  else {                                  // KHz
    display.print(F("KHz"));
    q=p-3;
    strcpy(Hertz,LCDstr);                 // get Herz digits (3)
    strcpy(LCDstr+q,Hertz+(q-1));         // copy into LCDstr and add to KHz
    LCDstr[q]='.';                        // decimal point
  }

  switch (p)
  {
    case 5:                               //  10 KHZ
      display.setCursor(36,0);
      break;
    case 6:                               // 100 KHZ
      display.setCursor(24,0);
      break;
    case 7:                               //   1 MHZ
      display.setCursor(12,0);
      break;
    case 8:                               //  10 MHZ
      display.setCursor(0,0);
      break;
    case 9:                               // 100 MHZ
      display.setCursor(0,0);
      break;
  }
    
  display.setTextSize(2);  
  display.println(LCDstr);
  display_settings();
}


/**************************************/
/* Displays step, mode and version    */
/**************************************/
void display_settings()
{
// Stepsize  
  display.setCursor(1, 42);  
  display.setTextSize(2);  
  display.print(F("Step"));
  switch (radix)
  {
    case 1:
      display.println(F("   1Hz"));
      break;
    case 10:
      display.println(F("  10Hz"));
      break;
    case 100:
      display.println(F(" 100Hz"));
      break;
    case 1000:
      display.println(F("  1kHz"));
      break;
    case 10000:
      display.println(F(" 10kHz"));
      break;
    case 100000:
      display.println(F("100kHz"));
      break;
    case 1000000:
      display.println(F("  1MHz"));
      break;
  }


  display.setCursor(12, 25);
  switch (disp_txt)
  {
    case 0:
      display.print(F("                 "));   // clear line    
      break;
    case 1:
      display.print(F("** Turn RIT Off *"));
      break;
    case 2:
      display.print(F("*** Set to TRx **"));
      break;
    case 3:
      display.print(F("** Calibration **"));
      break;      
    case 4:
      display.print(F("* Calibration OK!"));
      break;      
  }
  display.display();
}


/**************************************/
/*            S E T U P               */
/**************************************/
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)

  si5351.output_enable(SI5351_CLK0, 1); // 1 - Enables / 0 - Disables CLK
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA);  // Output current 2MA, 4MA, 6MA or 8MA

// Read EEPROM
  radix = eeprom_read_dword((const uint32_t *)EE_SAVED_RADIX);
  if ((radix < 10UL) | (radix > 1000000UL)) radix = 100UL;  
  
  vfo_t = eeprom_read_dword((const uint32_t *)EE_SAVED_AFREQ);
  if ((vfo_t < F_MIN) | (vfo_t > F_MAX)) vfo_t = 14000000ULL;



  vco_c = 0;
  
  xt_freq = SI5351_XTAL_FREQ + vco_c;

//initialize the Si5351
  si5351.set_correction(0); // Set to zero because I'm using an other calibration method
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, xt_freq);    // Frequency get from settings in VFO_si5351.h file
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  
// Set CLK0 to output the starting "vfo" frequency as set above by vfo = ?
  si5351.set_freq((vfo_t * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_2MA);
// Set CLK1 to output the Rx frequncy = vfo +/- bfo frequency
  if (vfo_t <= bfo_f) vfo_r = vfo_t + bfo_f;    // Upper / lower mixing  
  else vfo_r = vfo_t - bfo_f;    
  si5351.set_freq((vfo_r * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK1);
  si5351.drive_strength(SI5351_CLK1,SI5351_DRIVE_2MA);
// Set CLK2 to output bfo frequency
  si5351.set_freq((bfo_f * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK2);
  si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_2MA);

// Clear the buffer.
  display.clearDisplay();
  display.display();
  
// text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
// Encoder setup
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  PCICR |= (1 << PCIE2);                       // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);

  sei();

// Pin Setup
  pinMode(Calibrbtn, INPUT_PULLUP);   // Calibrate
  pinMode(RIT_Switch, INPUT_PULLUP);  // RIT Switch
  pinMode(TX_Switch, INPUT_PULLUP);   // Select TRx or BFO
  
// Display first time  
  display_frequency();  // Update the display
}

/**************************************/
/*             L O O P                */
/**************************************/
void loop()
{
  if (disp_txt == 4) {
    delay(3000);                  // Display calibration OK and wait 3 seconds
    disp_txt = 0;
  }

  get_button();
  
// Update the display if the frequency has been changed
  if (changed_f)  {
    display_frequency();

    if (act_clk == 0 && !calibrate)                   // No Tx update during calibrate
      si5351.set_freq((vfo_t * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0);
    else if (act_clk == 2)                            // BFO update
      si5351.set_freq((bfo_f * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK2);      
// Update Rx frequency    
    if (vfo_t <= bfo_f) vfo_r = vfo_t + bfo_f;      // Upper / lower mixing  
    else vfo_r = vfo_t - bfo_f;    
    si5351.set_freq((vfo_r * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK1);

    changed_f = 0;
    disp_txt = 0;                   // Clear line
  }


  
// Button press
// Also stored the last used frequency together with the step size before store
//
  if (get_button()) {
    if (stepflag) {                 // Stepsize button
      eeprom_write_dword((uint32_t *)EE_SAVED_RADIX, radix);  // Store frequency and stepsize
      eeprom_write_dword((uint32_t *)EE_SAVED_AFREQ, vfo_t);
  
      switch (radix)
      {
      case 1:
        radix = 10;
        break;
      case 10:
        radix = 100;
        break;
      case 100:
        radix = 1000;
        break;
      case 1000:
        radix = 10000;
        break;
      case 10000:
        radix = 100000;
        break;
      case 100000:
        radix = 1000000;
        break;
      case 1000000:
        radix = 10;
        break;       
      }
      stepflag  = 0;
    }
    else if (modeflag)  {         // Mode button
      if (act_clk == 0) act_clk = 2; else act_clk = 0;
      eeprom_write_dword((uint32_t *)EE_SAVED_BFREQ, bfo_f);
      modeflag = 0;  
      disp_txt = 0;                                 // Clear line
    }
    else if (calflag) {                             // Calibrate button
      if (!digitalRead(RIT_Switch)){                // RIT is on
        disp_txt = 1;                               // Message RIT off
      }
      else if (act_clk == 2){                       // BFO mode on
        disp_txt = 2;                               // Message BFO off        
      }
      else if (!calibrate)  {                       // Start calibrate
        vfo_s = vfo_t;                              // Save actual freq
        old_radix = radix;                          // and stepsize
        vfo_t = SI5351_XTAL_FREQ;                   // en set to default x-tal
        disp_txt = 3;                               // Message Calibrate
        calibrate = 1;
        radix = 10;                                 // Set to 10 Hz        
        si5351.set_freq((vfo_t * SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0); // Set CLK0
      }
      else if (calibrate) {                         // after tuning x-tal freq
        calibrate = 0;
        vco_c = vfo_t - SI5351_XTAL_FREQ;           // difference
        vfo_t = vfo_s;                              // restore freq
        radix = old_radix;                          // and stepsize
        disp_txt = 4;                               // Message Calibrate OK
        
        eeprom_write_dword((uint32_t *)EE_SAVED_OFSET, vco_c);        // store correction
        xt_freq = SI5351_XTAL_FREQ + vco_c;                           // Calibrated x-tal freq
        eeprom_write_dword((uint32_t *)EE_SAVED_CALBR, 0x60);         // Calibrated
        si5351.init(SI5351_CRYSTAL_LOAD_8PF, xt_freq);                // Initialize
        si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);        
        si5351.set_freq(bfo_f * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK2);   // correct BFO frequency
        si5351.set_freq(vfo_t * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK0);   // Correct Tx freq
        if (vfo_t <= bfo_f) vfo_r = vfo_t + bfo_f;                                  // Upper / lower mixing
          else vfo_r = vfo_t - bfo_f;
        si5351.set_freq(vfo_r * SI5351_FREQ_MULT, SI5351_PLL_FIXED, SI5351_CLK1);   // correct Rx frequency
      }
      calflag = 0;
    }
  }    
  display_frequency();                              // Update display
} // end while loop
