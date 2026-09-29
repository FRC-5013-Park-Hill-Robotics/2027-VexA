#include "main.h"
#include "lemlib/api.hpp"

#include <string>
#include <random>

#include "constants.h"
#include "TroLib/conveyor.h"
#include "TroLib/elevator.h"
#include "TroLib/piston.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

conveyor mIntake(ports_const::intake);
elevator mElevator(ports_const::left_elevator, ports_const::right_elevator);
piston mClaw('A');

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
                              lemlib::Omniwheel::NEW_275,
                              480, //Drivetrain rpm
							  2 //drift
							  );

pros::Imu imu(ports_const::imu);

pros::Rotation v_rotation_sensor(-6);
pros::Rotation h_rotation_sensor(-7);

lemlib::TrackingWheel vertical_tracking_wheel(&v_rotation_sensor, lemlib::Omniwheel::NEW_2, -0.3);
lemlib::TrackingWheel horizontal_tracking_wheel(&h_rotation_sensor, lemlib::Omniwheel::NEW_2, 2.5);

lemlib::OdomSensors sensors(nullptr,//&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            nullptr,//&horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
							&imu //inertia
							);


lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
						sensors // Odometry Sensors
						);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

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

enum type{
	Match,
	Skills,
	Test
};


// ----- ----- -----
// CHANGE 
// ----- ----- -----

type mType = Match;

void runSkill() {
	
}

void autonomous() {
	if(mType == Match){
		mElevator.run(1);
		chassis.moveToPoint(0, 5, 200, {.maxSpeed = 50}, false);
		chassis.moveToPoint(15, 14, 800, {.maxSpeed = 127}, true);
		pros::delay(400);
		mElevator.run(-1);
		pros::delay(900);
		mClaw.activate(true);
		chassis.moveToPoint(1, 14, 400, {.forwards = false, .maxSpeed = 127}, false);
		mElevator.run(0);
		chassis.turnToHeading(90, 700, {.maxSpeed = 50}, false);
		pros::delay(200);
		chassis.moveToPoint(0, -7, 1000, {.forwards = false,.maxSpeed = 127}, false);

		chassis.moveToPoint(11, 25, 1000, {.maxSpeed = 127}, false);
		chassis.moveToPoint(15, 29, 1000, {.maxSpeed = 50}, false);
		mClaw.activate(false);
		mElevator.run(1);
		pros::delay(200);
		chassis.turnToHeading(175, 700, {.maxSpeed = 50}, false);
		chassis.moveToPoint(34, 15, 1500, {.maxSpeed = 127}, true);
		pros::delay(900);
		mElevator.run(-1);
		pros::delay(500);
		mClaw.activate(true);

		pros::delay(200);
		mElevator.run(0);
		chassis.moveToPoint(34, 20, 1500, {.forwards = false, .maxSpeed = 127}, false);
		mElevator.run(-1);
		chassis.turnToHeading(140, 1000, {.maxSpeed = 50}, false);
		chassis.moveToPoint(47, 9, 1500, {.maxSpeed = 127}, false);
		chassis.moveToPoint(50, 6, 1500, {.maxSpeed = 50}, false);
		mClaw.activate(false);
		chassis.moveToPoint(55, 2, 1500, {.maxSpeed = 50}, false);
		mElevator.run(1);
		chassis.turnToHeading(270, 1000, {.maxSpeed = 50}, false);
		chassis.moveToPoint(35, 0, 1500, {.maxSpeed = 127}, true);
		pros::delay(800);
		mElevator.run(-1);
		pros::delay(600);
		mClaw.activate(true);
		mElevator.run(0);

		chassis.moveToPoint(45, 5, 1500, {.forwards = false, .maxSpeed = 127}, true);
	}
	if(mType == Skills){
		runSkill();
	}
	if(mType == Test){
		chassis.moveToPoint(0, 5, 1000, {.maxSpeed = 50}, false);
		chassis.turnToHeading(90, 700, {.maxSpeed = 50}, false);
		chassis.moveToPoint(5, 5, 1000, {.maxSpeed = 50}, false);
		chassis.turnToHeading(180, 700, {.maxSpeed = 50}, false);
		chassis.moveToPoint(5, 0, 1000, {.maxSpeed = 50}, false);
		chassis.turnToHeading(270, 700, {.maxSpeed = 50}, false);
		chassis.moveToPoint(0, 0, 1000, {.maxSpeed = 50}, false);
		chassis.turnToHeading(0, 700, {.maxSpeed = 50}, false);
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
	bool mUpLateState = false;
	bool mDownLateState = false;

	while (true) {
		//pros::lcd::set_text(0, "Elevator Position: " + std::to_string(mElevator.getCurrentPosition()));
		//pros::lcd::set_text(1, "Elevator Target: " + std::to_string(mElevator.getTargetPosition()));
		// pros::lcd::set_text(2, "Elevator Setpoint: " + std::to_string(mElevator.getCurrentSetpointIndex()));
		//pros::lcd::set_text(3, "GoToPose: " + std::to_string(mElevator.getTargetPosition()));

		//pros::lcd::print(1, "Rotation Sensor H: %i", h_rotation_sensor.get_position());
		//pros::lcd::print(2, "Rotation Sensor V: %i", v_rotation_sensor.get_position());

		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
		// 	mIntake.run(1);
		// }
		// else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
		// 	mIntake.run(-1);
		// }
		// else{
		// 	mIntake.run(0);
		// }

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			mClaw.toggle();
		}

		// Setpoint Elevator Control
		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
		// 	mElevator.incrementSetpoint(1);
		// }
		// else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
		// 	mElevator.incrementSetpoint(-1);
		// }
		// else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
		// 	mElevator.goToSetpoint(0);
		// }
		// mElevator.update();

		// Manual Elevator Control
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
			mElevator.run(1);
			mUpLateState = true;
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			mElevator.run(-1);
			mDownLateState = true;
		}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && mUpLateState){
			mElevator.run(0);
			mUpLateState = false;
		}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && mDownLateState){
			mElevator.run(0);
			mDownLateState = false;
		}
		
		pros::delay(20);
	}
}