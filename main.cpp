/** @file rudder-2020/main.cpp
    @brief Rudder node code for Sailbot AY20 Hull 14 mod 3
    L Marino and D Evangelista
*/

#include "mbed.h"
#include "nmea2k.h" // use dev branch!
#include "pgn/iso/Pgn60928.h" // ISO address claim
#include "pgn/Pgn126993.h" // heartbeat
#include "pgn/Pgn127245.h" // rudder
#include "hull14mod3.h"

#define RUDDER_VERSION "14.3.0 PT1"

Serial pc(USBTX,USBRX);
nmea2k::CANLayer n2k(p30,p29); // for sending nmea2k messages
unsigned char node_addr = HULL14MOD3_RUDDER_ADDR;
DigitalOut rxled(LED2);

// Consider abstracting all of this as a SailbotActuator
// This part should be abstracted as a Pst360
AnalogIn   r_ain(p15);
// This part should be abstracted as a Pololu
PwmOut  rudder( p22 );
DigitalOut   r_dir( p21 );
DigitalOut    r_I(p23);
DigitalOut r_slp(p30); //sleep
DigitalOut r_brk(p8);
float r_ang = 100;
float r_order = 180.0;
float RC_1;

float xx = 5.5; //changes the threshold that the motor goes to sleep on
float gg = 9.5; //changes the threshold that the motor goes to sleep on for mast
float zz = 121; //changes wait at end of rc thread
float ww = 5; //changes wait time at end of if statments in mast rudder threads
int ff = 65;//changes wait at end of telemetry

//**get position**
float posr();
// *****threading*****
Thread rudder_thread;

void rudder_process(void);

int main()
{
    nmea2k::Frame f;
    nmea2k::PduHeader h;

    // TODO startup ROS publisher LATER
    //nh.initNode();
    //nh.advertise(chatter);

    // startup messages
    pc.printf("0x%02x:main: Rudder node version %s\r\n",node_addr,RUDDER_VERSION);
    pc.printf("0x%02x:main: nmea2k version %s\r\n",node_addr,NMEA2K_VERSION);
   

    // Assert ISO address and wait

    // start the various processes
    rudder_thread.start(&rudder_process);
    
    pc.printf("0x%02x:main: listening for any pgn\r\n",node_addr);
    while (1) {

        if (n2k.read(f)) {
            rxled = 1;
            h = nmea2k::PduHeader(f.id);
            pc.printf("0x%02x:main: recieved priority %d, pgn %d, sa 0x%02x, da 0x%02x: 0x",node_addr,h.p(), h.pgn(), h.sa(), h.da());
            for (int i=0; i<f.len; i++)
                pc.printf("%02x",f.data[i]);
            pc.printf("\r\n");

            //First attempt at taking things from NMEA and putting it on ROS
            if((h.pgn()== 127245)&&(h.instance() == 0 ) { //see if rudder pgn and correct instance
		r_order = f.angle_order();  //Need to divide by resolution
		// e.g. (float) f.angle_order()/PGN_127245_ANGLE_RES;
		
            pc.printf("\r\n r_order: %f \r\n",r_order);                  //I forgot how to get data out of pgns..
            } //if(h.pgn()...


            rxled = 0;
        } // if (n2k.read(f))

        //nh.spinOnce();
        ThisThread::sleep_for(10);
    } // while(1)
    //I forgot how to get data out of pgns..
} //if(h.pgn()...


rxled = 0;
} // if (n2k.read(f))

//nh.spinOnce();
ThisThread::sleep_for(1000);
} // while(1)
} // int main(void)



// Remember this part is being handeld on the bridge node? 
void rudder_process(void)
{
    while(1) {
        
        r_pos = (r_ain-.108)/.002466;
        pc.printf("RC: %.1f\n", r_ang);

        if((r_pos > (r_order-xx)) && (r_pos < (r_order+xx))) {
            rudder.pulsewidth(0);
            r_slp = 0;
        }
        if( (r_pos > (r_order+xx)) ) {  //&& r_pos < 235.0
            r_slp = 1;
            r_dir = 1; //left??
            rudder.pulsewidth(.0005);
            Thread::wait(ww);
            r_pos = (r_ain-.108)/.002466;
        }//if pos
        if((r_pos < (r_order-xx)) ) {   // && r_pos > 55.0
            r_slp = 1;
            r_dir = 0; //right??
            rudder.pulsewidth(.0005);
            Thread::wait(ww);
            r_pos = (r_ain-.108)/.002466;
        }
        pc.printf("STEP: %.1f\n", r_pos);
        ThisThread::sleep_for(1000);
    }//while(1)
}//rudcallback




float posr()
{
    float r1;
    float r2;
    float r3;
    r1 = (r_ain-.108)/.002466;
    Thread::wait(3);
    r2 = (r_ain-.108)/.002466;
    Thread::wait(3);
    r3 = (r_ain-.108)/.002466;
    return (r1+r2+r3)/3.0;
}
