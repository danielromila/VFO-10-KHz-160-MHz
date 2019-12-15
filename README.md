# VFO-10-KHz-160-MHz
VFO 10 KHz to 160 MHz with Arduino Nano, Si 5351 and SSD1306
You can see it in function at: https://www.youtube.com/watch?v=7KEPbQaxyXg

  I (Daniel Romila) simplified this program in order to use it as a simple, single OUTPUT VFO
  from 10 KHz to 160 MHz.

  It is necessary to declare, in the old style, the OLED display as 128 x 64, in the Adafruit_SSD1306.h, from the 
  Adafruit_SSD1306 library. Just take out the comments (the //) from in front of: #define SSD1306_128_64
  
  A part of this program is taken from Jason Mildrum, NT7S.
  All extra functions are written by Rob Engberts PA0RWE
  
  UPDATE Dec 15 2019: You should use directly the second version of the code, written by LA3PNA. It works from 1 MHz up.
