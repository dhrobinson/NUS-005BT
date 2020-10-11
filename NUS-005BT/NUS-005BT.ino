#include <Joystick.h>
Joystick_ Joystick;
/**
 * Gamecube controller to Nintendo 64 adapter
 * by Andrew Brown
 * Rewritten for N64 to HID by Peter Den Hartog
 * 
 * Output HID data via SoftwareSerial for bluetooth 
 * by David Robinson
 * 
 * Hat tip to whiterabbit for joystick auto calibration
 */

/**
 * To use, hook up the following to Arduino (16mhz)
 * Digital I/O 2: N64 serial line
 * All appropriate grounding and power lines
 */

#include "pins_arduino.h"
#include <SoftwareSerial.h>  

int bluetoothTx = 14;  // TX-O pin of RN42
int bluetoothRx = 16;  // RX-I pin of RN42

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx); 

// default min/max values for joystick
int axis_x_min = -70;
int axis_x_max = 70;
int axis_y_min = -70;
int axis_y_max = 70;
int axis_x_val, axis_y_val;

#define N64_PIN 2 // Arduino pin to receive controller data
#define N64_PIN_REGISTER 0x02 // Important: hardware address for the pin

// these two macros set arduino pin 2 to input or output, which with an
// external 1K pull-up resistor to the 3.3V rail, is like pulling it high or
// low.  These operations translate to 1 op code, which takes 2 cycles
#define N64_HIGH DDRD &= ~N64_PIN_REGISTER
#define N64_LOW DDRD |= N64_PIN_REGISTER
#define N64_QUERY (PIND & N64_PIN_REGISTER)

// 8 bytes of data that we get from the controller
struct {
    // bits: 0, 0, 0, start, y, x, b, a
    unsigned char data1;
    // bits: 1, L, R, Z, Dup, Ddown, Dright, Dleft
    unsigned char data2;
    char stick_x;
    char stick_y;
} N64_status;
char N64_raw_dump[33]; // 1 received bit per byte


void N64_send(unsigned char *buffer, char length);
void N64_get();
void print_N64_status();
void translate_raw_data();

#include "crc_table.h"

void setup()
{
  Joystick.begin(0);
  Serial.begin(115200);
  bluetooth.begin(57600);  // The Bluetooth Mate defaults to 115200bps but I set it to 57.6k

  // Communication with gamecube controller on this pin
  // Don't remove these lines, we don't want to push +5V to the controller
  digitalWrite(N64_PIN, LOW);  
  pinMode(N64_PIN, INPUT);

  /*
  Uncomment the code below to use a Gamecube controller (untested)
  */

  /*
  Initialize the gamecube controller by sending it a null byte.
  This is unnecessary for a standard controller, but is required for the
  Wavebird.
  */
  
  /*
  unsigned char initialize = 0x00;
  noInterrupts();
  N64_send(&initialize, 1);
  */

  /*
  // Stupid routine to wait for the gamecube controller to stop
  // sending its response. We don't care what it is, but we
  // can't start asking for status if it's still responding
  int x;
  for (x=0; x<64; x++) {
      // make sure the line is idle for 64 iterations, should
      // be plenty.
      if (!N64_QUERY)
          x = 0;
  }
  */

  /*
  // Query for the gamecube controller's status. We do this
  // to get the 0 point for the control stick.
  unsigned char command[] = {0x01};
  N64_send(command, 1);
  // read in data and dump it to N64_raw_dump
  N64_get();
  interrupts();
  translate_raw_data();  
  */
}

void translate_raw_data()
{
    // The get_N64_status function sloppily dumps its data 1 bit per byte
    // into the get_status_extended char array. It's our job to go through
    // that and put each piece neatly into the struct N64_status
    int i;
    memset(&N64_status, 0, sizeof(N64_status));
    // line 1
    // bits: A, B, Z, Start, Dup, Ddown, Dleft, Dright
    for (i=0; i<8; i++) {
        N64_status.data1 |= N64_raw_dump[i] ? (0x80 >> i) : 0;
    }
    // line 2
    // bits: 0, 0, L, R, Cup, Cdown, Cleft, Cright
    for (i=0; i<8; i++) {
        N64_status.data2 |= N64_raw_dump[8+i] ? (0x80 >> i) : 0;
    }
    // line 3
    // bits: joystick x value
    // These are 8 bit values centered at 0x80 (128)
    for (i=0; i<8; i++) {
        N64_status.stick_x |= N64_raw_dump[16+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        N64_status.stick_y |= N64_raw_dump[24+i] ? (0x80 >> i) : 0;
    }
}


/**
 * This sends the given byte sequence to the controller
 * length must be at least 1
 * Oh, it destroys the buffer passed in as it writes it
 */
void N64_send(unsigned char *buffer, char length)
{
    // Send these bytes
    char bits;
    
    //bool bit;

    // This routine is very carefully timed by examining the assembly output.
    // Do not change any statements, it could throw the timings off
    //
    // We get 16 cycles per microsecond, which should be plenty, but we need to
    // be conservative. Most assembly ops take 1 cycle, but a few take 2
    //
    // I use manually constructed for-loops out of gotos so I have more control
    // over the outputted assembly. I can insert nops where it was impossible
    // with a for loop
    
    asm volatile ("#Starting outer for loop");
outer_loop:
    {
        asm volatile ("#Starting inner for loop");
        bits=8;
inner_loop:
        {
            // Starting a bit, set the line low
            asm volatile ("#Setting line to low");
            N64_LOW; // 1 op, 2 cycles

            asm volatile ("#branching");
            if (*buffer >> 7) {
                asm volatile ("#Bit is a 1");
                // 1 bit
                // remain low for 1us, then go high for 3us
                // nop block 1
                asm volatile ("nop\nnop\nnop\nnop\nnop\n");
                
                asm volatile ("#Setting line to high");
                N64_HIGH;

                // nop block 2
                // we'll wait only 2us to sync up with both conditions
                // at the bottom of the if statement
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              );

            } else {
                asm volatile ("#Bit is a 0");
                // 0 bit
                // remain low for 3us, then go high for 1us
                // nop block 3
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\n");

                asm volatile ("#Setting line to high");
                N64_HIGH;

                // wait for 1us
                asm volatile ("#end of conditional branch, need to wait 1us more before next bit");
                
            }
            // end of the if, the line is high and needs to remain
            // high for exactly 16 more cycles, regardless of the previous
            // branch path

            asm volatile ("#finishing inner loop body");
            --bits;
            if (bits != 0) {
                // nop block 4
                // this block is why a for loop was impossible
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\n");
                // rotate bits
                asm volatile ("#rotating out bits");
                *buffer <<= 1;

                goto inner_loop;
            } // fall out of inner loop
        }
        asm volatile ("#continuing outer loop");
        // In this case: the inner loop exits and the outer loop iterates,
        // there are /exactly/ 16 cycles taken up by the necessary operations.
        // So no nops are needed here (that was lucky!)
        --length;
        if (length != 0) {
            ++buffer;
            goto outer_loop;
        } // fall out of outer loop
    }

    // send a single stop (1) bit
    // nop block 5
    asm volatile ("nop\nnop\nnop\nnop\n");
    N64_LOW;
    // wait 1 us, 16 cycles, then raise the line 
    // 16-2=14
    // nop block 6
    asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\n");
                  
    N64_HIGH;
}

void N64_get()
{
    // listen for the expected 8 bytes of data back from the controller and
    // blast it out to the N64_raw_dump array, one bit per byte for extra speed.
    // Afterwards, call translate_raw_data() to interpret the raw data and pack
    // it into the N64_status struct.
    asm volatile ("#Starting to listen");
    unsigned char timeout;
    char bitcount = 32;
    char *bitbin = N64_raw_dump;

    // Again, using gotos here to make the assembly more predictable and
    // optimization easier (please don't kill me)
read_loop:
    timeout = 0x3f;
    // wait for line to go low
    while (N64_QUERY) {
        if (!--timeout)
            return;
    }
    // wait approx 2us and poll the line
    asm volatile (
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
            );
    *bitbin = N64_QUERY;
    ++bitbin;
    --bitcount;
    if (bitcount == 0)
        return;

    // wait for line to go high again
    // it may already be high, so this should just drop through
    timeout = 0x3f;
    while (!N64_QUERY) {
        if (!--timeout)
            return;
    }
    goto read_loop;
}

/**
 * Write the controller status out through SoftwareSerial for the RN42
 * https://stackoverflow.com/questions/27661297/hid-reports-scan-codes-for-rn-42-hids-gamepad-profile/28528348
 */
void sendGamepadState(uint8_t btnState1, uint8_t btnState2, int8_t x1, int8_t y1)
{
  bluetooth.write((byte)0xFD);    // Start HID Report
  bluetooth.write((byte)0x6);     // Length byte
  
  // 1st X/Y-Axis
  bluetooth.write(x1);            // First X coordinate
  bluetooth.write(y1);            // First Y coordinate
  
  // 2nd X/Y-Axis; write null because we don't use a second joystick
  bluetooth.write((uint8_t)0x00); //Second X coordinate
  bluetooth.write((uint8_t)0x00); //Second Y coordinate
  
  // Buttons
  bluetooth.write(btnState1);     // Second Byte (Buttons 1-8)
  bluetooth.write(btnState2);     // Second Byte (Buttons 9-16)
}

void print_N64_status()
{
    // bits: A, B, Z, Start, Dup, Ddown, Dleft, Dright
    // bits: 0, 0, L, R, Cup, Cdown, Cleft, Cright
    Serial.println();
    Serial.print("Start: ");
    Serial.println(N64_status.data1 & 16 ? 1:0);

    Serial.print("Z:     ");
    Serial.println(N64_status.data1 & 32 ? 1:0);

    Serial.print("B:     ");
    Serial.println(N64_status.data1 & 64 ? 1:0);
    Serial.print("A:     ");
    Serial.println(N64_status.data1 & 128 ? 1:0);
    Serial.print("L:     ");
    Serial.println(N64_status.data2 & 32 ? 1:0);
    Serial.print("R:     ");
    Serial.println(N64_status.data2 & 16 ? 1:0);

    Serial.print("Cup:   ");
    Serial.println(N64_status.data2 & 0x08 ? 1:0);
    Serial.print("Cdown: ");
    Serial.println(N64_status.data2 & 0x04 ? 1:0);
    Serial.print("Cright:");
    Serial.println(N64_status.data2 & 0x01 ? 1:0);
    Serial.print("Cleft: ");
    Serial.println(N64_status.data2 & 0x02 ? 1:0);
    
    Serial.print("Dup:   ");
    Serial.println(N64_status.data1 & 0x08 ? 1:0);
    Serial.print("Ddown: ");
    Serial.println(N64_status.data1 & 0x04 ? 1:0);
    Serial.print("Dright:");
    Serial.println(N64_status.data1 & 0x01 ? 1:0);
    Serial.print("Dleft: ");
    Serial.println(N64_status.data1 & 0x02 ? 1:0);

    Serial.print("Stick X:");
    Serial.println(N64_status.stick_x, DEC);
    Serial.print("Stick Y:");
    Serial.println(N64_status.stick_y, DEC);
    
    Serial.print("Stick Reset:");
    Serial.println(N64_status.data2 & 128 ? 1:0);
}

void loop()
{
    // Command to send to the gamecube
    // The last bit is rumble, flip it to rumble
    // yes this does need to be inside the loop, the
    // array gets mutilated when it goes through N64_send
    unsigned char command[] = {0x01};

    // don't want interrupts getting in the way
    noInterrupts();
    
    // send those 3 bytes
    N64_send(command, 1);
    
    // read in data and dump it to N64_raw_dump
    N64_get();
    
    // end of time sensitive code
    interrupts();

    // translate the data in N64_raw_dump to something useful
    translate_raw_data();

    // If the reset button combo is pressed, unpress L & R
    if(N64_status.data2 & 128)
    {
      N64_status.data2 &= ~(1UL << 4);
      N64_status.data2 &= ~(1UL << 5);
    }

    /*
    // Joystick
    axis_x_val = (int) (N64_status.stick_x, DEC);
    axis_y_val = (int) (N64_status.stick_y, DEC);

    // Track the x/y axis to find its min/max values for auto calibration
    if (axis_x_val > 0 && axis_x_val > axis_x_max) {
      axis_x_max = axis_x_val;
    } else if (axis_x_val < 0 && axis_x_val < axis_x_min) {
      axis_x_min = axis_x_val;
    }
    */
    /*
    if(axis_x_val > axis_x_max){
      axis_x_max = axis_x_val;
    }
    if(axis_y_val < axis_y_min){
      axis_y_min = axis_y_val;
    }
    if(axis_y_val > axis_y_max){
      axis_y_max = axis_y_val;
    }
    */

    // Rewrite the axis to a % of 127
    //int axis_x = map(axis_x_val, axis_x_min, axis_x_max, 0, 256);
    //int axis_y = map(axis_y_val, axis_y_min, axis_y_max, 0, 256);
    int axis_x = map(axis_x_val, -80, 85, -127, 127);
    int axis_y = map(axis_y_val, -80, 85, -127, 127);

    // Write the status out
    sendGamepadState(N64_status.data1, N64_status.data2, axis_x, axis_y * -1);

    Joystick.setXAxis(N64_status.stick_x);
    Joystick.setYAxis(N64_status.stick_y * -1);
    Joystick.sendState();

    // DEBUG
    Serial.println();
    Serial.print(axis_x_min);
    Serial.print(" ");
    Serial.print(N64_status.stick_x, DEC);
    Serial.print(":");
    Serial.print(axis_x_val);
    Serial.print(":");
    Serial.print(axis_x);
    Serial.print(" ");
    Serial.println(axis_x_max);
    
    Serial.print(axis_y_min);
    Serial.print(" ");
    Serial.print(N64_status.stick_y, DEC);
    Serial.print(":");
    Serial.print(axis_y_val);
    Serial.print(":");
    Serial.print(axis_y);
    Serial.print(" ");
    Serial.println(axis_y_max);
    print_N64_status();
    
    delay(25);
}
