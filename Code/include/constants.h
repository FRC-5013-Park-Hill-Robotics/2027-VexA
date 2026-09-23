#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

namespace ports_const{
    //left drive: 
    //right drive: 
    const unsigned int imu = 13;

    const unsigned int right_elevator = 20;
    const unsigned int left_elevator = 18;

    const unsigned int intake = 19;

};

namespace drive_const{
    const pros::MotorGears gearset = pros::MotorGears::blue;
}

namespace elevator_const{
    const pros::MotorGears gearset = pros::MotorGears::blue;
    const pros::MotorEncoderUnits units = pros::MotorEncoderUnits::degrees; //Setpoints in degrees
    const int speed = 100;
    const int slow_speed = 50;
};

namespace conveyor_const{
    const pros::MotorGears gearset = pros::MotorGears::blue;
    const pros::MotorEncoderUnits units = pros::MotorEncoderUnits::degrees; //Setpoints in degrees
    const int speed = 600;
    const int slow_speed = 350;
};

namespace feedout_const{
    const pros::MotorGears gearset = pros::MotorGears::blue;
    const pros::MotorEncoderUnits units = pros::MotorEncoderUnits::degrees; //Setpoints in degrees
    const int speed = 600;
};

namespace color_const{
    const int red_hue = 26;
    const int blue_hue = 120;
    const int red_tolerance = 10;
    const int blue_tolerance = 65;
};

namespace other_const{
    const std::string quotes[] = {
        "I never feel more alone than when I'm trying to put sunscreen on my back. -Jimmy Kimmel",
        "Whoever established the high road and \nhow high it should be should be fired. -Sandra Bullock",
        "I refuse to join any club that would \nhave me as a member. -Groucho Marx",
        "If you can't be kind, at least be vague.\n -Judith Martin",
        "I admit that my level of weirdness is \nabove the national average, but I'm comfortable with that. -@Rebel Circus",
        "\"Hey, what kind of belt do you have?\" \n\"Canvas. You Like? J.C. Penny. $3.98.\" -Karate Kid"
    };
};

#endif
