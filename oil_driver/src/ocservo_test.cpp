
#include "ocservo_driver.h"

using namespace std;
using namespace LinuxSerialPort;

int main(int argc, char** argv )
{
    ocservo::OCServoRS485 servo(1, "servo");

    servo._goto_position(0.64);

//    while (true) {
//        servo.spin_once();
//    }

    servo.serialport_->Close();
}