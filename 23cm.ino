
/* This code is used in conjunction with PE1JPD's 23cm transceiver
 * (http://www.pe1jpd.nl/index.php/23cm_nbfm)
 * and is based on his original C/AVR code. 
 * 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 
 * PLEASE, PLEASE read the README.1ST file on how to wire an Ardiono Mini
 * to the original standalone ATMega328 28p DIL socket!!
 * 
 * Again, PLEASE read the README.1ST file, otherwise this code is useless!
 * 
 * Also look at the JPG file Arduino-Nano-Mini-pinmapping.jpg in this repo.
 * 
 * Needless to say, but always necessary, use this code at your own risk.
 * 
 * Ported/rewritten to/for the Arduino IDE by PA3FYM.
 *
 * v0.1 March 4 2016 Initial release by PA3FYM
 *   
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * 
 * This software is released under the 'Beerware' license.
 * As long as you retain this notice you can do whatever you want with this stuff. 
 * If we meet some day, and you think this stuff is worth it, you buy me a beer in return.
 * Remco PA3FYM
 * 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

#include <LiquidCrystal.h>
#include <EEPROM.h> 

#define debug  // some ifdefs in the source to chit-chat a little over
               // the serial/USB bus

// I/O ports to control 'inside' of the transceiver
const byte mute =   A0; // mute RX audio
const byte Smeter = A1; // RSSI pin of MC3362
const byte txon =   A2; // switches TX part of transceiver
const byte clk =    A3; // ADF4113HV clock 
const byte data =   A4; // ADF4113HV data
const byte le =     A5; // ADF4113HV latch enable

// I/O ports for 'outside' controls, all active low
const byte rotary = 3;      // rotary switch INT1
const byte rotary2 = 2;     // other rotary switch
const byte rotary_push = 9; // push button on rotary encoder
const byte ptt = 8;         // PTT (Push To Talk) 
const byte ctcss_pin = 13;  // ctcss signal pin

int8_t rot_dir;             // rotary encoder direction, 0 = no action, 1 = CW, 255 = CCW


// all frequencies are in kHz (extra 0's for Hz doesn't make sense with 25 kHz raster)
uint32_t  if_freq;         // 1st IF (80 - 10.7 MHz) of receiver in kHz.
uint32_t  fref;            // PLL reference frequency in kHz (max. Fref for ADF4113HV = 150 MHz)
uint16_t  fraster = 25;    // raster frequency in kHz
uint32_t  freq;            // frequency in kHz  <-- float takes 1.5 kB memory extra!!

int16_t   shifts[] = {0,-6000,-28000}; // common repeater shifts in kHz in Europe, 0 = no shift (simplex)


uint8_t  sig_level;     // relative signal strength
int8_t   squelch_level; // squelch level (0 - 9)
uint16_t bucket;        // sig level 'damper'' 
uint8_t  escape;

boolean  tx = false; // if tx = false then receive, if tx = true then transmit
int8_t   last=0;
uint32_t passed; // elapsed time

/* ctcss tones *10 (Hz) or 'time constants' for the Timer1 ISR.
 *  
 * E.g. 88.5 Hz ctcss --> 1/88.5 = 11.3ms period
 * 50% duty cycle means 11.3/2  = 5.65 ms low, and 5.65 ms high.
 * 
 * Timer1 is filled with a value so that when it runs empty
 * an interrupt is generated, resulting in toggling the ctcss audio pin (pin13). 
*/
uint16_t tones[] = {0,670,693,719,744,770,797,825,854,885,915,948,974,1000,1035,1072,1109,
                    1148,1188,1230,1273,1318,1365,1413,1462,1514,1567,1622,1679,1738,1799,
                    1862,1928,2035,2107,2181,2257,2336,2418,2503}; // 40 entries (0 - 39)                  

uint16_t count1; // Timer1 counter value


 
byte block[8][8]=
{
  { 0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10 },  // define character for fill S-character
  { 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18 },
  { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C },
  { 0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E },

  { 0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08 },  // define character for peak S-character
  { 0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04 },
  { 0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02 },
  { 0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01 },
};



LiquidCrystal lcd(11,10,4,5,6,7);  // RS, EN, D4, D5, D6, D7  initialize LCD display

void init_pll() { //initialize PLL, raster is (still) fixed to 25 kHz

#ifdef debug
  Serial.println("PLL initializing ...");
#endif

  PORTC &= B11000111; //make LE, DATA, CLK (PD5,4,3) low 

  // First xelect function latch map with F1 bit set, see page 13 ADF4113HV datasheet
  // - Set function latch           (bits 0,1):
  // - Hold R/A/B counters on reset (F1, bit 2 = 1)
  // - Normal power                 (bit 3)
  // - Three state muxout           (bit 4,5,6)
  // - Positive Phase detection     (bit 7)
  // - Three-state Charge pump      (bit 8)
  // - High current Charge pump     (bits 15,16,17)
  // - Prescaler 16/17              (bits 22,23)
  
  //  Result of all this: B010000111000000010000110 = 0x438086; 
 
  writePLL(0x438086); // clock in Function Latch Map information, see page 13 datasheet
    
  // Next, load reference counter latch, see bottom of page 11 ADF4113HV datasheet
  // Set reference counter latch     (bit 0 and bit1 = 0):
  // 7.2ns anti-backlash pulse width (bit 16,17) <-- we set bit17 !
  // Divide ratio                    (bits 2 to 15) <-- calc, calc .. but then shift left 2 twice !
  
  writePLL(0x020000 + (fref/fraster) << 2); // set R counter (Note! 0x020000 = bit17 set!)

  // Now load AB counter, see page 12 ADF4113HV datasheet 

  // freq = 1299000; // use this freq as it's 3rd harmonic of 433.000 (handy with 70cm handy ;-)
  
 freq = EEPROMreadlong(0x00);           //get last stored frequency 

 #ifdef debug
  Serial.println(freq);
 #endif
 
  if (freq < 1240000 || freq > 1300000) { // if eeprom contents are out of range
      freq = 1298375;                     // e.g. first startup with this software 
      EEPROMwritelong(0x00,freq);         // store it with defaults
  }

 // set_freq(freq - if_freq); // fill AB counter latch with desired PLL frequency

  // Select function latch map with F1 bit cleared
  // See page 13 ADF4113H datasheet
  // Set function latch (bits 0,1):
  // - Normal R/A/B counters on reset (F1, bit 2 = 0)
  // - Normal power                   (bit 3)
  // - Three state muxout             (bit 4,5,6)
  // - Positive Phase detection       (bit 7)
  // - Three-state Charge pump        (bit 8)
  // - High current Charge pump       (bits 15,16,17)
  // - Prescaler 16/17                (bits 22,23)
  
  //  Result of all this: B010000111000000010000010 = 0x438082;
  
  writePLL(0x438082); // clock in Function Latch Map with F1=0, meaning: ready to go !!
}

void set_freq(uint32_t freq) {          // set PLL frequency with loading AB counter
  
  uint16_t channel = freq/fraster;      // calculate 'channel number'
  
                          //----------------- PLL prescaler value (16 in this case)                
  uint32_t B = (channel / 16) & 0x1fff; // mask 13 bits in B
  uint16_t A = (channel % 16) & 0x3f;   // mask  7 bits in A

  uint32_t AB = 1 + (B << 8) + (A << 2);// first '1' = bit0 = C1 , i.e. select AB counter latch                               
                                        // shift B 8 positions to load B in bits 13-8
                                        // shift A 2 positions to load A in bits 7-2  
  writePLL(AB);                         // now program AB counter
  
#ifdef debug
  Serial.print("Frequentie: ");Serial.print(freq);Serial.print(", kanaal: ");Serial.println(channel);
#endif
}


void writePLL(uint32_t pll_word) {      // this routine clocks PLL word (24 bits long) into the PLL
                                        // from msb (bit23) to lsb (bit0)

#ifdef debug
   Serial.print("Input:    ");Serial.println(pll_word,BIN);
   Serial.print("PLL word: ");
#endif
 
  for (uint8_t flop=0; flop<24; flop++) {          // PLL word has 24 bits
    digitalWrite(data,(pll_word & 0x800000? 1:0)); // AND with MSB 
    
#ifdef debug    
    Serial.print(pll_word & 0x800000? 1:0);
#endif

    digitalWrite(clk,1);                           // clock in bit on rising edge of CLK
    digitalWrite(clk,0);
    
    pll_word <<= 1; // rotate left to next bit
  }

#ifdef debug
    Serial.println();
#endif

    digitalWrite(le,1);                            // latch in data on rising edge of LE
    digitalWrite(le,0);
}

void setup () { 
  
digitalWrite(mute,0); // first thing to do: mute audio

Serial.begin(115200);

/*  below 'ports and pinmodes' are set with 'mnemonics'
 *  which is shorter code and very specific for the hardware
 *  
 *  do NOT change this, unless you know what you're doing!
 *  
 *  and .. even then ... do NOT change this!
 */
 
DDRC   = B00111101; // PORTC PC0,2-5 output, PC1 input (S-meter), PC7-6 'do not exist'
PORTC |= B00000001; // mute receiver directly

DDRD  |= B11110000; // PD7-PD4 LCD display outputs, leave PD0-3 untouched
DDRD  &= B11110011; // PD3 and PD2 are inputs
PORTD |= B00001100; // in any case set pull up resistors for PD3-2 (rotary encoder)

DDRB  &= B11111100; // PB1,0 are inputs, leave PB7-6 untouched
DDRB  |= B00101100; // all PB5,3,2 ports are outputs, leave PB7-6 untouched, PB4 no designator yet
PORTB |= B00011111; // set pull up resistors for PB5-PB0, leave PB7-6 untouched, PB5 (ctcss) = low

lcd.begin(16,2);    // we have a 16 column, 2 row LCD display
lcd.print("Hello Dude!"); //

for( int i=0 ; i<8 ; i++) lcd.createChar(i,block[i]);  // create charcters for S-meter

delay(1000);
lcd.clear();

defaults();         // first fetch last settings from eeprom or store defaults in eeprom
init_Timer1();      // initialize and -if appropriate- start Timer1
init_pll();         // initialize PLL
refresh();          // build up main LCD screen after startup

attachInterrupt(digitalPinToInterrupt(rotary),int1_isr,FALLING); // assign INT1 (rotary encoder)
                                                                 // use falling edge ( = active low)
passed = millis();  // get time stamp after reboot
}

void loop() {                         // main loop 

int8_t tune;
     
    tx = !digitalRead(ptt);            // poll PTT pin
    tune = rot_dial();                 // poll rotary encoder
    
     if (tx) { 
                                       // arrive here when PTT is pressed                             
         digitalWrite(mute,1);         // first mute the receiver
         
         digitalWrite(txon,1);         // switch on TX part
         
          if (tx != last) {            // if last status was 0 then RX --> TX transition
            Serial.println("Transmit!!");
            refresh();                 // PLL is programmed once, so not every poll cycle, certainly not during TX !
            last = tx;                 // last status = tx (= 1)
          }                                          
     }
     
     else  {                                          // PTT is released or not pressed, in other words, we are in RX mode

            sig_level = rssi();                       // fetch signal strength
            if (!squelch_level) digitalWrite(mute,0); // mute audio depending on squelch level and signal strength
            else digitalWrite(mute,(squelch_level > sig_level/10)); // 0 = open, 1 = audio mute

            displayS();                               // display relative signal strength on lower row 
             
            if (last > 0) {                           // if last status is 1, this indicates TX --> RX transition
              digitalWrite(txon,0);                   // switch off TX part
              digitalWrite(ctcss_pin,0);              // during RX ctcss pin always 0
              Serial.println("Receive!!");
              refresh();                              // the PLL is programmed only once, so not every poll cycle
              last = 0;                               // reset last status to 0 ( = RX)
            }
           
            if (tune) {
               if (tune > 0) freq += fraster; else freq -= fraster; // tune frequency
               refresh();                             // refresh display
            }
        
            if (millis() - passed > 10000) {           // check if 10 secs passed
              passed = millis();                       // if yes, update 
              EEPROMwritelong(0x00,freq);              // update last freq if necessary
              Serial.println("Freq updated !! (if necessary)");
              }  

            if (rot_push()) menu(); // go to settings menu when rotary push button is pressed      
      }
}

void int1_isr() { // INT1 ISR, arrive here on FALLING edge of PD3 (one switch of rotary encoder)
                  // rot_dir has to be zero and check for status of PD2 (other rotary switch)
                  
    //if (digitalRead(rotary2)) rot_dir=255; else rot_dir=1; delay(10);       
    if (PIND & B00000100) rot_dir=255; else rot_dir=1; //delay(20); // rot_dir < 0 when anti clockwise, > 0 when clockwise
}

void defaults() {                             // get default/startup values from EEPROM:
                                              // 0x00 - 0x03 = frequency in kHz
                                              // 0x04 - 0x07 = PLL ref frequency in kHz
                                              // 0x0b        = TX shift number
                                              // 0x0e        = ctcss tone number
                                              // 0x10        = squelch level (0 - 9)
                                              // 0x14        = 1st IF frequency in kHz
uint8_t i;

    fref = EEPROMreadlong(0x04);              // get stored Fref
    //Serial.println(fref);
    if (fref < 2000 || fref > 150000 || fref%100 != 0) {  // check valid boundaries
        fref = 12000;                         // fref must be multiple of 100 kHz
        EEPROMwritelong(0x04,fref);           // store default Fref (12 MHz)
        }
        
    i = EEPROM.read(0x0b);                    // get stored shift number 
    if (i < 0 || i > 2) {                     // if out of range then
      i = 2;                                  // i = 2 --> TX shift = -28 MHz = Dutch 23cm repeater shift
      EEPROM.update(0x0b,i);                  // store shift number
    }

     i = EEPROM.read(0x0e);                   // get stored ctcss tone number 
     if (i < 0 || i > 39) {                   // check valid boundaries
         EEPROM.update(0x0e,0);               // if invalid/no tone nr stored --> tone_nr = 0
         TCCR1B = 0;                          // in this case stop Timer1
     }
     else calc_count1(i);                     // calculate Timer1 counter value from tone number
               
     squelch_level = EEPROM.read(0x10);        // squelch level is uint8 --> 1 byte
   // Serial.println(squelch_level);
     if (squelch_level < 0 || squelch_level > 9) { // sq level must be between 0 - 9
      squelch_level = 0;
      EEPROM.update(0x10,squelch_level);
     }

     if_freq = EEPROMreadlong(0x14);             // get stored IF freq
     if (if_freq < 28000 || if_freq > 150000 || if_freq%100 != 0) {  // check valid boundaries
      if_freq = 69300;                           // if not in range, set default IF = 69.3 MHz
      EEPROMwritelong(0x14,if_freq);             // ... and store IF freq
     }

}

void refresh() {                       // refreshes display info and programs PLL
   uint16_t kHz;
   
   lcd.setCursor(0,0);                 // select top line, first position 
   
   if (tx) {                           // TX active ?
    kHz = EEPROM.read(0x0b);           // read TX shift nr from EEPROM
    set_freq(freq+shifts[kHz]);        // program PLL with TX frequency (= RX freq + shift)
    Serial.println(freq+shifts[kHz]);  // debug chit-chat
    lcd.setCursor(0,1);                // select lower row, first position
    lcd.print("                ");     // clear lower row
   }
   else set_freq(freq-if_freq);        // if RX then program PLL with RX frequency - 1st IF freq
   
    
    lcd.setCursor(0,0);                // select top line, first position 
    lcd.print(tx? (freq+shifts[kHz])/1000 : freq/1000);lcd.print("."); // print frequency in MHz.
    
    kHz = freq % 1000;                 // isolate kHz

    if (!kHz) lcd.print("00");         // if e.g. 1297.000 MHz add '00'
    else if (kHz < 100) lcd.print("0");// if kHz < 100 add '0'
    
    lcd.print(kHz);                    // print kHz portion of frequency 
    
    lcd.print(" MHz  "); lcd.print(tx?"TX":"RX"); // tail with remaining characters  
}

int16_t rssi() {                       // poll analog pin A1 (PC1) connected to rssi pin MC3362

uint8_t sig;
                                        //--- this value determined so that with only noise sig = ca. 0
        sig = (1023-analogRead(Smeter))-54; // weak signal adc = 1023, strong adc = 0

        if (sig < 0) sig = 0;             // negative s values do not exist ;-)

        //the 3 lines below form a simple low pass filter to 'damp' the s values a little
                                          // JPD's low pass filter gives some 'damping' of the s outcome indeed :-)
        sig += bucket;                    // 'damp' S-meter signal so that S-meter falls slowly
        sig >>= 1;                        // divide by 2, so that during polling values will not 'flicker' 
        bucket = sig;                     // store in bucket for next poll cycle
    
        return sig;                       // publish result
}

int rot_dial() {                          // INT1 ISR handles rotary ports PD3 and PD2 <-- INT1

        int8_t flop;
   
        flop = rot_dir;                   // get value obtained from INT1 ISR
        rot_dir = 0;                      // reset rot_dir for INT1 ISR routine
        
        return flop;                      // 0x01 if clockwise, 0xff anti clockwise, 0 if nothing
}

int rot_push() {                          // rotary push button pressed?

        if (!digitalRead(rotary_push)) {  // poll I/O port

               while (!digitalRead(rotary_push)) {delay(20);} // hang while pressed
                      
               return true;                // yes
        }
        return false;                      // no
}

int ptt_press() {                          // ptt pressed?/

          if (!digitalRead(ptt)) {         // poll I/O port

               while (!digitalRead(ptt)) {delay(20);} // hang while pressed
                      
               return true;                // yes
        }
        return false;                      // no   
}

void menu() {                             // with rotary dial select menu item, push to change item
  
  int8_t flop,item=0;

        lcd.clear(); 

        while (!escape) {
           lcd.setCursor(0,0);
           lcd.print("Menu: "); 
           menu_item(item);               // display item

           flop = rot_dial();
           
           if (flop) {        
             if (flop < 0) item--; else item++; 
             if (item < 0) item = 5;      // keep items within boundaries
             if (item > 5) item = 0;
            } 
          } // while !escape
        escape=0;                         // reset escape and return to main loop
}      

void menu_item(uint8_t item) {            // deal with submenus

int8_t flap,i;

    lcd.setCursor(6,0);
    
    switch (item) {

          case 0:
             lcd.print("Squelch ");
              if (rot_push()) {           // poll rotary push button 
                                          // if pushed ... 
                while (!rot_push()) {     // hang until pushed again
                 flap = rot_dial();
                  if (flap) {             // if flap <> 0 , thus rotary dial ... is dialed ;-)
                   
                   if (flap > 0) squelch_level++; else squelch_level--;
                   if (squelch_level < 0) squelch_level = 0; // 10 levels are enough
                   if (squelch_level > 9) squelch_level = 9;
                   delay(20);             // effort to debounce encoder
                  }

                 lcd.setCursor(0,1);lcd.print(squelch_level);   // print squelch level in lower row
#ifdef debug                 
                 Serial.print(rssi());Serial.print(" ");Serial.println(squelch_level);
                 delay(50);
#endif
                 if (!squelch_level) digitalWrite(mute,0); // if level = 0 , open audio ( = noise ...)
                 else digitalWrite(mute,(squelch_level > rssi()/10)); // else mute audio
                 } // while !rot_push
                 
                 lcd.clear();                     // clear display 
                 EEPROM.update(0x10,squelch_level);   // ... and update (new) level in EEPROM     
                } // if rot_push       
          break; // end of item 0 (squelch)

          case 1:
             lcd.print("TX Shift");
             
             if (rot_push()) {                    // poll rotary push button
              i = EEPROM.read(0x0b);              // if pressed, read TX shift nr from EEPROM
              
              while (!rot_push()) {               // hang until rotary push button is pressed
                flap = rot_dial();                // poll rotary dial
                if (flap) {
                  
                   if (flap > 0) i++; else i--;   // i points to entries in shifts[] array
                   if (i < 0) i = 2;              // keep TX shift within boundaries
                   if (i > 2) i = 0;
                   delay(20);
                }
                lcd.setCursor(0,1);
                if (!i) lcd.print("simplex ");
                else {
                  if (i<2) lcd.print(" ");        // if -6 MHz print preceding " "
                  lcd.print(shifts[i]/1000);lcd.print(" MHz");
                }
              } // while
              lcd.clear();                         // clear LCD
              EEPROM.update(0x0b,i);               // store shift nr in EEPROM
             }
          break;

          case 2:
             lcd.print("Ctcss   ");
             
             if (rot_push()) {
              i = EEPROM.read(0x0e);                              // 3rd byte in ctcss long contains tone number
 
              while (!rot_push()) {                               // pushing the rotary button exits
                flap = rot_dial();                                // get rotary information
                
                if (flap) {
                   if (flap > 0) i++; else i--;                   // same type of code as in other menu items
                   if (i > 39) i = 0;                             // tones[] has 40 entries (0 - 39)
                   if (i < 0) i = 39; 
                   
            //       Serial.print(tones[i]);Serial.print(" ");Serial.println(i);
                   delay(20);
                }
                lcd.setCursor(0,1);
                 if (!i) lcd.print("  off    ");                   // i = 0 = no ctcss tone
                 else { 
                   if (i < 13) lcd.print(" ");                     // if tone < 13 (100 Hz) print preceding ' '
                   lcd.print(tones[i]/10);lcd.print(".");          // print selected ctcss tone, e.g. 88.5 Hz
                   lcd.print(tones[i]%10);lcd.print(" Hz  ");
                 }
              } // while
              lcd.clear();                                     // clear LCD
                                          
              if (i) {                                         // if ctcss tone selected (i > 0) then
                calc_count1(i);                                // . . .calculate Timer1 counter value
                TCCR1B = 2;                                    // and . . . start Timer1
              }
               else TCCR1B = 0;                                // when no ctcss don't start or stop Timer1 
               
           //   Serial.print("Tone nr for TCCR1B : ");Serial.println(i);    
              EEPROM.update(0x0e,i);                               // store tone number
             }   
          break;

          case 3:
             lcd.print("PLL Fref");                             // ADF4113HV Fref up to 150 MHz possible

             
             if (rot_push()) {                                  // poll rotary push button
                 
              while (!rot_push()) {                             // button pressed, now hang until rotary push button is pressed again
                flap = rot_dial();                              // poll rotary dial 
                if (flap) {
                   if (flap > 0) fref += 100; else fref -= 100; // 100 kHz per step
                   delay(20);
                }   
                
                lcd.setCursor(0,1);                             // cursor to 1st column lower row 
                lcd.print(fref);lcd.print(" kHz   ");           // print fref in kHz
                     
              } // while !rot_push
              
              lcd.clear();                                      // clear display
              EEPROMwritelong(0x04,fref);                       // store selected Fref in eeprom
              init_pll();                                       // initialize PLL with (new) reference frequency
             } // if rot_push
          break;

          case 4:
             lcd.print("First IF");                             
             if (rot_push()) {                                  // poll rotary push button
              
              while (!rot_push()) {                             // if pushed keep hanging until another push
                flap = rot_dial();
                if (flap) {                                     // if flap <> 0 then rotary dial is ... dialed ;-)
                  
                   if (flap > 0) if_freq +=100; else if_freq -=100; // 1st IF freq goes in 100 kHz portions
                   delay(50);
                }
                lcd.setCursor(0,1);                             // point to col 0 , lowest row
                lcd.print(if_freq/1000);lcd.print(".");         // print MHz + '.'
                lcd.print(if_freq%1000);                        // print kHz portion 
                lcd.print(" MHz  ");                            // followed with 'MHz' 
              } // while
              
              lcd.clear();
              EEPROMwritelong(0x14,if_freq);      // store IF freq in eeprom
             }
          break;
       
          case 5:
             lcd.print("Exit    ");
             if (rot_push()) {
              escape++;                      // set escape to fall into the main loop
              Serial.println("Escape!!");
              refresh();                     // refresh display to 'normal'
              }
          break;          
    }
}

void EEPROMwritelong(uint16_t address, int32_t value) { // stores a long int (32 bits) into EEPROM
                                                        // byte3 = MSByte, byte0 = LSByte
uint8_t flop;
int32_t oldvalue;                                       // to enhance life time of EEPROM :-)

      oldvalue = EEPROMreadlong(address);               // get contents
      
      if (value != oldvalue) {                          // only write when new value <> old value
        for (flop=0 ; flop<4 ; flop++) EEPROM.write(address+flop,(value >> flop*8)); // <- automatic & 0xff due to EEPROM byte :-)
      }
}

int32_t EEPROMreadlong(uint16_t address) {              // reads and returns long int (32 bits) from EEPROM

       uint8_t byte0 = EEPROM.read(address);
      uint16_t byte1 = EEPROM.read(address + 1);
      uint32_t byte2 = EEPROM.read(address + 2);
      uint32_t byte3 = EEPROM.read(address + 3);

      return byte0 + (byte1 << 8) + (byte2 << 16) + (byte3 << 24); // reassemble 32 bits word
}

ISR(TIMER1_OVF_vect) {                     // Timer1 ISR, i.e. when Timer1 resets this routine is called

        TCNT1 = count1;                    // on arrival reload Timer 1 counter first 
      //  Serial.println(count1);
            
        if (tx) PINB = bit(5);             // toggle PB5 = ctcss pin only during TX
     // if (tx) PORTB = PORTB ^ B00100000; // ^^^^\__ above line is 4 bytes shorter :-)                
}

uint16_t calc_count1(uint8_t i) {          // calculate and load Timer1 value derived from ctcss tones

/*
 * Okay, Arduino clock speed is 16 MHz and Timer1 has a 16 bits counter.
 * If prescaler is 1 (TCCR1B = 0x01) Timer1 has a resolution of 1/16e6 = 62.5 ns / tick.
 *
 * Suppose we want to generate 100 Hz ctcss. 
 * This means the ctcss pin has to toggle twice as fast, i.e. 200 Hz
 * 200 Hz = 1/200 sec = 5 ms. In this 5 ms are 5e-3/62.5e-9 = 80000 ticks.
 *
 * 80000 ticks do not fit within 16 bits. So, this is the reason the 8 prescaler is activated (TCCR1B = 0x02)
 *
 * With the 8 prescaler the resolution is 8/16e6 = 0.5 us , which is accurate enough
 *
 * We enter this routine with the ctcss tone number. E.g. 0 = 0 Hz = no ctcss, 9 = 88.5 Hz
 *
 * After some maths it can be derived that the amount of counter ticks = 1e7/tone-frequency
 *
*/ 

  if (i) count1 = 65536 - (1e7/tones[i]);    // Timer1 value, resolution 0.5 us / tick
                                             // btw this calculation takes 842 bytes (!)
                                             // so, perhaps write a shorter routine? 
                                             
  else TCCR1B = 0;                           // when no ctcss (i = 0) disable Timer1                                            
  }

void init_Timer1() { // deliberately chosen NOT to include TimerOne.h to have shorter code
                     //
                     // The Timer1 setup process is explained in the link below
                     // https://arduinodiy.wordpress.com/2012/02/28/timer-interrupts 

     noInterrupts();                // disable interrupts
 
     TCCR1A = 0;                    // initialize Timer/Counter Control Registers (TCCR) for Timer1 , default = 0
     TCCR1B = 0;                    // 0 = stop Timer1
                  

     TIMSK1 |= (1 << TOIE1);        // generate Timer1 interrupt when counter overflows
     TCNT1= count1;                 // fill counter 
     if (count1 != 0) TCCR1B = 2;   // if ctcss then start Timer1 and use 8 prescaler, i.e. resolution = 0.5 us
 
     interrupts();                  // enable interrupts 
}

void displayS() {                   // display relative S-signalbar in the lower row during RX

int8_t   level,maxlevel,dlay;       // level max memory, delay & speed for peak return
uint32_t lastT=0;

byte  fill[]={0x20,0x00,0x01,0x02,0x03,0xff};      // character used for fill 
byte  peak[]={0x20,0x00,0x04,0x05,0x06,0x07,0x20}; // character used for peak 


#define t_refresh    100            // msec bar refresh rate
#define t_peakhold   5*t_refresh    // msec peak hold time before return

  if (millis() < lastT) return;     // determine 1 ms time stamps
  lastT += t_refresh;   
 
  level = sig_level << 1;           // max rssi() is 73 (@FYM), so multiply with 2 
                                    // for full scale (estimated guess ;-)                                
    
  lcd.setCursor(0,1);               // select lower row
  lcd.write("S ");                  // 'S'peaks for itself ;-)
  
  for (uint8_t i=2 ; i<16 ; i++) {  // we have 14 columns left
  
    int8_t f = constrain(level    -i*5,0,5 ); // constrain values 
    int8_t p = constrain(maxlevel -i*5,0,6 );
    
    if (f) lcd.write(fill[f]); else lcd.write(peak[p]); // depending on phenomenon write bars on the lower row
  }
  
  if (level > maxlevel) {            // if current level > maxlevel
       
    maxlevel = level;                // save max level
    dlay = -t_peakhold/t_refresh;    // Starting delay value. negative = peak is stable
  }
  
  else {                             // we have a lower level than (former) maxlevel
  
    if (dlay > 0) maxlevel -= dlay;  // speed up attack time when signal increases

    if (maxlevel < 0) maxlevel=0; else dlay++; // keep maxlevel >=0 , and when maxlevel decays increase delay
  }
}



