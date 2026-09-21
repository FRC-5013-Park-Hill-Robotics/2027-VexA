#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

namespace ports_const{
    //left drive: 7, 19, 18
    //right drive: 10, 9, 20
    const unsigned int imu = 17;
    const unsigned int tunnel = 1;
    const unsigned int elevator = 2;
};

namespace drive_const{
    const pros::MotorGears gearset = pros::MotorGears::blue;
}

namespace tunnel_const{
    const pros::MotorGears gearset = pros::MotorGears::blue;
    const pros::MotorEncoderUnits units = pros::MotorEncoderUnits::degrees; //Setpoints in degrees
    const int speed = 600;
    const int slow_speed = 350;
};

namespace elevator_const{
    const pros::MotorGears gearset = pros::MotorGears::blue;
    const pros::MotorEncoderUnits units = pros::MotorEncoderUnits::degrees; //Setpoints in degrees
    const int speed = 600;
};

namespace other_const{
    const std::string quotes[] = {
        "I never feel more alone than when \nI'm trying to put sunscreen on my back. -Jimmy Kimmel",
        "Whoever established the high road and \nhow high it should be should be fired. -Sandra Bullock",
        "I refuse to join any club that would \nhave me as a member. -Groucho Marx",
        "If you can't be kind, at least be vague.\n -Judith Martin",
        "I admit that my level of weirdness is \nabove the national average, \nbut I'm comfortable with that. -@Rebel Circus",
        "\"Hey, what kind of belt do you have?\" \n\"Canvas. You Like? J.C. Penny. $3.98.\" -Karate Kid"
    };
};

#endif
