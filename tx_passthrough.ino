/*
READ DATA FROM TX.
---
PINS
  EXTERNAL POWER REQUIRED
  D3 -> SIGNAL (TX)
---
CHANNEL DESCRIPTION
  CH1 -> Yaw (0-100%)
  CH2 -> Pitch (0-100%)
  CH3 -> Throttle (0-100%)
  CH4 -> Roll (0-100%)
  CH5 -> VAA (0-100%)
  CH6 -> VAB (0-100%)
  CH7 -> SWD (0, 1)
  CH8 -> SWC (0, 1, 2)
  CH9 -> SWB (0,1)
  CH10 -> SWA (0,1)
*/

#include <SoftwareSerial.h>
#include <Servo.h>

Servo rudder;
Servo elevator;
Servo aileron_l;
Servo aileron_r;

#define IBUS_BUFFSIZE 32 // 32 bytes expected
#define IBUS_MAXCHANNELS 10  // 10 channels expected

//SoftwareSerial iBusSerial(3, 4); // rx on D3. D4 unused.

static uint8_t ibusIndex = 0; // ticker variable
static uint8_t ibus[IBUS_BUFFSIZE] = {0}; // buffer list
static uint16_t rcValue[IBUS_MAXCHANNELS] = {0}; // postprocessed list

void setup() {
  rudder.attach(12);
  elevator.attach(11);
  aileron_l.attach(10);
  aileron_r.attach(9);

  Serial.begin(115200); // serial monitor comms
  //iBusSerial.begin(115200); // ibus comms

  Serial.println("FlySky iBus Signal Monitor Started!");
  // rudder.write(120);

}

void loop() {
    readRx();
    //delay(200);
    rudder.write(rcValue[0]);
    aileron_l.write(rcValue[1]);
    aileron_r.write(rcValue[2]);
    elevator.write(rcValue[3]);

}

// Read iBus Data from Receiver
void readRx() {
    while (Serial.available()) {
        uint8_t val = Serial.read(); // read packet stream from ibus

        // if end sequence recieved, end function call
        if ((ibusIndex == 0 && val != 0x20) || (ibusIndex == 1 && val != 0x40)) { 
            ibusIndex = 0;
            return;
        }

        // Store received bytes in buffer
        if (ibusIndex < IBUS_BUFFSIZE) {
            ibus[ibusIndex++] = val;
        }

        // when buffer is full, process data
        if (ibusIndex == IBUS_BUFFSIZE) {
            ibusIndex = 0;

            // Extract channel values
            for (int i = 0; i < IBUS_MAXCHANNELS; i++) {
                rcValue[i] = (ibus[3 + (i * 2)] << 8) | ibus[2 + (i * 2)]; // some bs byte conversion thing. takes a byte, converts to int.
                rcValue[i] = constrain(rcValue[i], 1000, 2000); // clamps reading just in case
                
                // post-processing 
                // throttle inputs. map from PWM to 0-100%
                /*
                if (i <=5){
                  rcValue[i] = map(rcValue[i], 1000, 2000, 0, 100);
                }
                // boolean inputs. map from PWM to 0-1
                else if (i != 7){
                  rcValue[i] = map(rcValue[i], 1000, 2000, 0, 1);
                }
                // SWC, which is a 3-step switch. map from PWM to 0,1,2
                else{
                  rcValue[i] = map(rcValue[i], 1000, 2000, 0, 2);
                }
                */
            }
            //   // display to serial monitor
            // Serial.print("Channels: ");
            // for (int i = 0; i < IBUS_MAXCHANNELS; i++) {
            //     Serial.print(rcValue[i]);
            //     Serial.print(" ");
            // }
            // Serial.println();

        }
    }
}