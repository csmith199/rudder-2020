#include "mbed.h"
#include "nmea2k.h" // use dev branch!
#include "pgn/iso/Pgn60928.h" // ISO address claim
#include "pgn/Pgn126993.h" // heartbeat
#include "pgn/Pgn127245.h" // mast
#include "hull14mod3.h"

#define BRIDGE_VERSION "14.3.0 PT1"
int i;
Serial pc(USBTX,USBRX);
nmea2k::CANLayer n2k(p30,p29); // for sending nmea2k messages
unsigned char node_addr = HULL14MOD3_MAST_ADDR;
DigitalOut rxled(LED2);
DigitalOut txled(LED1);

AnalogIn   m_ain(p15);
PwmOut  mast( p22 );
DigitalOut   m_dir( p21 );
DigitalOut m_slp(p23); //sleep
DigitalOut m_brk(p8);
float m_pos = 100;
float m_order = 180.0;
float RC_1;

float xx = 5.5; //changes the threshold that the motor goes to sleep on
float gg = 9.5; //changes the threshold that the motor goes to sleep on for mast
float zz = 121; //changes wait at end of rc thread
float ww = 5; //changes wait time at end of if statments in mast mast threads
int ff = 65;//changes wait at end of telemetry

//**get position**
float posr();
// *****threading*****
Thread mast_thread;
Thread heartbeat_thread;

void mast_process(void);
void heartbeat_process(void);

int main(void)
{
    int read_interval = 1;
    
    nmea2k::Frame f;
    nmea2k::PduHeader h;
    nmea2k::Pgn127245 d(0,0,0,0);

    pc.printf("0x%02x:main: nmea2k version %s\r\n",node_addr,NMEA2K_VERSION);
    pc.printf("0x%02x:main: PGN 127245 receive demo\r\n",node_addr);

    heartbeat_thread.start(&heartbeat_process);
    mast_thread.start(&mast_process);
    pc.printf("0x%02x:main: listening for mast PGN 127245\r\n", node_addr);
    while (1) {

        if (n2k.read(f)) {
            h = nmea2k::PduHeader(f.id);
            if ((h.da() == NMEA2K_BROADCAST) || (h.da() == node_addr))
                switch(h.pgn()) {
                    case 127245:
                        //debug("0x%02x:main: handling mast PGN 127245\r\n", node_addr);
                        //d = PgnParser127245(f);
                        d = nmea2k::Pgn127245(f.data);
                        //debug("0x%02x:main: received data 0x",node_addr);
                        //for (int i=0; i<8; i++)
                        //  debug("%02x",d.data()[i]);
                        //debug("\r\n");
                        pc.printf("0x%02x:main: recieved %s, instance %d, direction_order %d, angle_order %3.1f, position %3.1f\r\n",
                                  node_addr,
                                  d.name,
                                  d.instance(),
                                  d.direction_order(),
                                  (float)d.angle_order()/PGN_127245_ANGLE_RES*180.0/NMEA2K_PI,
                                  (float)d.position()/PGN_127245_ANGLE_RES*180.0/NMEA2K_PI);
                        if(d.instance() == 1){
                        m_order = (float)d.angle_order()/PGN_127245_ANGLE_RES*180.0/NMEA2K_PI;
                        //pc.printf("m_order: %3.1f\r\n",m_order);
                        }//if(d.instance..
                        
                        break;
                    default:
                        pc.printf("0x%02x:main: received unhandled PGN %d\r\n",
                                  node_addr,h.pgn());
                } // switch(h.pgn())
        } // if addressed to us

        ThisThread::sleep_for(read_interval*100);
    } // while(1)
} // int main(void)








void mast_process(void)
{
    int direction = 0;
    float error = 0.0;
    float threshold = 6.0;
    float mast_interval = .2;
    mast.pulsewidth(0);
    mast.period(.001);
    pc.printf("mast process\r\n");
    
    while(1) {

        m_pos = posr();
       // pc.printf("mast pos: %f ", m_pos);

        error = m_pos - m_order;
        pc.printf("error: %f\r\n",error);

        while((abs(error)-threshold) > 0.0) {
            
            
            pc.printf("threshold: %f",abs(abs(error)-threshold));
            m_brk = 1;
            m_slp = 1;
            //ThisThread::sleep_for(1); //add a little time for break to engage
            direction = (int)(-1*(error/abs(error)));
            
            if(direction == -1) {
                direction = 0;
            }
            
            pc.printf("dir: %d\r\n",direction);
            m_dir = direction;
            mast.pulsewidth(.0005);
            
            m_pos = posr();
            error = m_pos - m_order;
            pc.printf("error: %f\r\n",error);
        }

        m_brk = 0;
        m_slp = 0;
        mast.pulsewidth(0);
        ThisThread::sleep_for(mast_interval*100);

    }

}//rudcallback

void heartbeat_process(void)
{
    nmea2k::Frame m;
    nmea2k::PduHeader h;
    nmea2k::Pgn126993 d(6000,0);
    unsigned int heartbeat_interval=60;
    unsigned char c=0;

    pc.printf("0x%02x:heartbeat_thread: starting heartbeat_process\r\n",
              node_addr);

    while(1) {
        h = nmea2k::PduHeader(d.p,d.pgn,node_addr,NMEA2K_BROADCAST);
        d = nmea2k::Pgn126993(heartbeat_interval*100,c++);
        m = nmea2k::Frame(h.id(),d.data(),d.dlen);
        if (n2k.write(m)) {
            txled = 1;
            pc.printf("0x%02x:heartbeat_thread: sent %s, %0.0f s, count %d\r\n",
                      node_addr,
                      d.name,
                      (float) d.update_rate()/100.0,
                      d.heartbeat_sequence_counter());
            ThisThread::sleep_for(5);
            txled = 0;
        } else
            pc.printf("0x%02x:heartbeat_thread: failed sending %s\r\n",
                      node_addr,
                      d.name);
        ThisThread::sleep_for(heartbeat_interval*1000);
    } // while(1)
} // void heartbeat_process(void)


float posr()
{
    float r1;
    float r2;
    float r3;
    r1 = (m_ain-.108)/.002466;
    Thread::wait(1);
    r2 = (m_ain-.108)/.002466;
    return (r1+r2)/2.0;
}
