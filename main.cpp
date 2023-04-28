#include "mbed.h"
#include "PinDetect.h"
#include "success.h"
#include "fail.h"
#include "LSM9DS1.h"
#include "mpr121.h"
#include "XNucleo53L0A1.h"
#include "uLCD_4DGL.h"
#undef I2C

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

InterruptIn interrupt(p8);
LSM9DS1 imu(p9, p10, 0xD6, 0x3C);
I2C i2c(p9, p10);
Mpr121 mpr121(&i2c, Mpr121::ADD_VSS);
DigitalIn inputSwitch(p12);
uLCD_4DGL uLCD(p13,p14,p15);
AnalogOut speaker(p18);
AnalogIn mypotentiometer(p19);
PinDetect pb(p20);
DevI2C *device_i2c = new DevI2C(p28, p27);
DigitalOut shdn(p29);

int status;

static XNucleo53L0A1 *board=NULL;

Serial pc(USBTX, USBRX);

Timer timer;
Ticker sampletick;
int game_state = 0;
int score = 0;
int timeout = 5;

class microphone
{
public :
    microphone(PinName pin);
    float read();
    operator float ();
private :
    AnalogIn _pin;
};
microphone::microphone (PinName pin):
    _pin(pin)
{
}
float microphone::read()
{
    return _pin.read();
}
inline microphone::operator float ()
{
    return _pin.read();
}
microphone mymicrophone(p16);

int key_code=0;
// Key hit/release interrupt routine
void fallInterrupt() {
  int i=0;
  int value=mpr121.read(0x00);
  value +=mpr121.read(0x01)<<8;
  // LED demo mod
  i=0;
  // puts key number out to LEDs for demo
  for (i=0; i<12; i++) {
  if (((value>>i)&0x01)==1) key_code=i+1;
  }
  led4=key_code & 0x01;
  led3=(key_code>>1) & 0x01;
  led2=(key_code>>2) & 0x01;
  led1=(key_code>>3) & 0x01;

}


#define sample_freq 11025.0
//get and set the frequency from wav conversion tool GUI
int i=0;
void play_fail_sound ()
{
    speaker.write_u16(fail_sound_data[i]);
    i++;
    if (i>= FAIL_NUM_ELEMENTS) {
        i = 0;
        sampletick.detach();
    }
}
void play_success_sound ()
{
    speaker.write_u16(success_sound_data[i]);
    i++;
    if (i>= SUCCESS_NUM_ELEMENTS) {
        i = 0;
        sampletick.detach();
    }
}

void set_score(int s) {
    if (s != 0) {
        sampletick.attach(&play_success_sound, 4.0 / sample_freq);
        wait(4.0);
    }
    uLCD.text_width(1); //4X size text
    uLCD.text_height(1);
    uLCD.locate(0,0);

    uLCD.printf("Score: %d", s);

}

void pb_hit_callback (void) {
    if (game_state == 1) {
        game_state = 2;
        srand((unsigned) time(NULL));
        score = 0;
        set_score(0);
    } else {
        game_state = 0;
    }
}

void tilt() {
    uLCD.text_width(2); // 4X size text
    uLCD.text_height(2);
    uLCD.locate(1, 5);
    uLCD.printf("Tilt!");
    timer.start();
    while (timer.read() < timeout) {
        imu.readAccel();
        pc.printf("accel: %9f %9f %9f in Gs\n\r", imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
        if (abs(imu.calcAccel(imu.ax)) > 0.3 || abs(imu.calcAccel(imu.ay)) > 0.3) {
            set_score(++score);
            return;
        }
    }
    game_state = 3;
}

void press() {
    int number = rand() % 12;
    uLCD.text_width(2); // 4X size text
    uLCD.text_height(2);
    uLCD.locate(1, 5);
    uLCD.printf("Press %d!", number);
    timer.start();
    while (timer.read() < timeout) {
        if (key_code == number + 1) {
            set_score(++score);
            return;
        }
    }
    game_state = 3;
}

void speak() {
    uLCD.text_width(6); //4X size text
    uLCD.text_height(6);
    uLCD.locate(1,5);

    uLCD.printf("Speak!");
    timer.start();
    while (timer.read() < timeout) {
        float tmp = mymicrophone;
        pc.printf("speak: %f\n", tmp);
        if (tmp > 0.5) {
            set_score(++score);
            return;
        }
    }
    game_state = 3;
}

//Rewrite using timeout
void cover() {
    uint32_t distance;
    uLCD.text_width(2); //4X size text
    uLCD.text_height(2);
    uLCD.locate(1,5);

    uLCD.printf("Cover!");

    wait(1);

    status = board->sensor_centre->get_distance(&distance);
    pc.printf("cover: %d\n", distance);
    if (status == VL53L0X_ERROR_NONE) {
        if (distance <= 100) {
            set_score(++score);
        } else {
            game_state = 3;
        }
    } else {
        uLCD.printf("ERROR");
        game_state = 1;
    }

}

//rewrite using timeout
void turn() {
    float temp = mypotentiometer;
    pc.printf("turn: %2f\n", temp);
    uLCD.text_width(2); //4X size text
    uLCD.text_height(2);
    uLCD.locate(1,5);

    uLCD.printf("Turn!");
    wait(1);
    float temp2 = mypotentiometer;
    pc.printf("turn: %2f\n", temp2);
    if (abs(temp2 - temp) > 0.3) {
        set_score(++score);
    } else {
        game_state = 3;
    }
}

void doSwitch() {
    int temp = inputSwitch;
    uLCD.text_width(6); //4X size text
    uLCD.text_height(6);
    uLCD.locate(1,5);

    uLCD.printf("Switch!");
    wait(1);
    if (inputSwitch != temp) {
        set_score(++score);
    } else {
        game_state = 3;
    }
}

int main() {
    int instruction;
    pc.printf("program started\n");
    pb.mode(PullUp);
    wait(.01);
    pb.attach_deasserted(&pb_hit_callback);
    pb.setSampleFrequency();
    imu.begin();
    if (!imu.begin()) {
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    imu.calibrate(1);
    
    board = XNucleo53L0A1::instance(device_i2c, NC, NC, NC);
    shdn = 0; //must reset sensor for an mbed reset to work
    wait(0.1);
    shdn = 1;
    wait(0.1);
    status = board->init_board();

    while (status) {
        pc.printf("Failed to init board! \r\n");
        status = board->init_board();
    }
    
    interrupt.fall(&fallInterrupt);
    interrupt.mode(PullUp);

    while(1) {
        if (game_state == 0) {
            pc.printf("game state: 0\n");
            score = 0;
            
            uLCD.cls();
            uLCD.text_width(2); //4X size text
            uLCD.text_height(2);
            uLCD.color(WHITE);
            uLCD.locate(1,3);

            uLCD.printf("Do It!");

            uLCD.text_width(1); //4X size text
            uLCD.text_height(1);
            uLCD.color(WHITE);
            uLCD.locate(1,6);

            uLCD.printf("Press Button");

            game_state = 1;
        }

        if (game_state == 1) {
            pc.printf("game state: 1\n");
            while (game_state == 1) {
                wait(0.1);
            }
        }

        if (game_state == 2) {
            while (game_state == 2) {
                pc.printf("game state: 2\n");
                instruction = 1 + rand() % 5;
                timer.reset();
                // Decrease timeout

                switch (instruction) {
                    case 1: tilt(); break;
                    case 2: press(); break;
                    case 3: speak(); break;
                    case 4: cover(); break;
                    case 5: turn(); break;
                }
            }
        }

        if (game_state == 3) {
            pc.printf("game state: 3\n");
            sampletick.attach(&play_fail_sound, 2.0 / sample_freq);
            uLCD.cls();
            uLCD.text_width(2); //4X size text
            uLCD.text_height(2);
            uLCD.locate(1,3);

            uLCD.printf("Game Over");

            uLCD.text_width(1); //4X size text
            uLCD.text_height(1);
            uLCD.locate(1,6);

            uLCD.printf("Score: %d", score);
            wait(2.0);
            game_state = 0;
        }
    }
}
