#include "main.h"
#include "lemlib/api.hpp"

#include <string>
#include <random>

#include "constants.h"
#include "conveyor.h"

#include "elevator.h"

#include "piston.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

conveyor mIntake(ports_const::intake);
elevator mElevator(ports_const::left_elevator, ports_const::right_elevator);

lemlib::ControllerSettings lateral_controller(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
	);

	// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
    	                                      0, // integral gain (kI)
        	                                  5, // derivative gain (kD)
            	                              3, // anti windup
                	                          .5, // small error range, in degrees
                    		                  100, // small error range timeout, in milliseconds
                    	                      1, // large error range, in degrees
                        	                  500, // large error range timeout, in milliseconds
                            	              0 // maximum acceleration (slew)
);
//-7 18 19
//9 -10 20  
pros::MotorGroup left_mg({-14, -15, -16}, drive_const::gearset);    // Creates a motor group 
pros::MotorGroup right_mg({11, 12, 17}, drive_const::gearset);  // + for forwards, - for reversed

lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.5, // track width (in)
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              300, //Drivetrain rpm
							  2 //drift
							  );

pros::Imu imu(ports_const::imu);

// pros::Rotation h_rotation_sensor(5);
// pros::Rotation v_rotation_sensor(4);

// lemlib::TrackingWheel horizontal_tracking_wheel(&h_rotation_sensor, lemlib::Omniwheel::NEW_2, 3.5);
// lemlib::TrackingWheel vertical_tracking_wheel(&v_rotation_sensor, lemlib::Omniwheel::NEW_2, 0);

lemlib::OdomSensors sensors(nullptr, //&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            nullptr, //&horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
							&imu //inertia
							);


lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
						sensors // Odometry Sensors
						);

enum Color{
	Red,
	Blue,
	None
};

enum Allignment{
	Left,
	Right,
	Inches,
	SoloWin
};

enum Type{
	Match,
	Skills,
	Test
};

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void displayOnLCD(const std::string& input) {
    std::istringstream iss(input);
    std::vector<std::string> words;
    std::string word;

    // Split input into words
    while (iss >> word) {
        words.push_back(word);
    }

    int maxLineLength = 39;
    std::string currentLine;
    int lineNumber = 1;

    for (size_t i = 0; i < words.size(); ++i) {
        // If adding this word exceeds maxLineLength
        if (!currentLine.empty() && (currentLine.length() + 1 + words[i].length() > maxLineLength)) {
            // Display current line
            pros::lcd::set_text(lineNumber, currentLine);
            lineNumber++;
            currentLine = words[i]; // start new line with current word
        } else {
            // Add space if not the first word in line
            if (!currentLine.empty()) {
                currentLine += " ";
            }
            currentLine += words[i];
        }
    }

    // Display any remaining text in currentLine
    if (!currentLine.empty()) {
        pros::lcd::set_text(lineNumber, currentLine);
    }
}

void initialize() {
	pros::lcd::initialize();

	// std::string text = other_const::quotes[controller.get_battery_level()%6];
	// displayOnLCD(text);

	//pros::lcd::set_text(4, random()%sizeof(other_const::quotes) / sizeof(other_const::quotes[0]));

	chassis.calibrate();
	chassis.setPose(0, 0, 0);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

Type mType = Match;
Color mColor = Red;
Allignment mAllignment = Left;

void runSkill() {
	
}


void autonomous() {
	if(mType == Match){
		int n = 0;
		if(mAllignment == SoloWin){
			n = -1;
		}
		if(mAllignment == Right){
			n = 1;
		}
		if(mAllignment == Left){
			n = -1;
		}

		if(mAllignment == Inches){
			chassis.moveToPoint(0, -5, 2000, {.forwards = false}, false);
		}
		
		else if(mAllignment == Left || mAllignment == Right){
			
		}
		else if(mAllignment == SoloWin){

		}
			
	}
	if(mType == Skills){
		runSkill();
	}
	if(mType == Test){
		//chassis.turnToHeading(270, 1800, {.maxSpeed = 50}, false);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		//pros::lcd::print(1, "V Rotation Sensor: %i", v_rotation_sensor.get_position());
		//pros::lcd::print(2, "Elevator Pose: ", mElevator.getCurrentPosition());

		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			mIntake.run(1);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			mIntake.run(-1);
		}
		else{
			mIntake.run(0);
		}

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
			mElevator.run(1);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			mElevator.run(-1);
		}
		else{
			mElevator.run(0);
		}

		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
		// 	mElevator.goToPosition(1);
		// }
		// else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
		// 	mElevator.goToPosition(-1);
		// }
		// mElevator.update();
		
		pros::delay(20);
	}
}