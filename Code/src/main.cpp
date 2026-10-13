#include "main.h"
#include "lemlib/api.hpp"

#include <string>
#include <random>

#include "constants.h"
#include "TroLib/conveyor.h"
#include "TroLib/elevator.h"
#include "TroLib/piston.h"
#include "TroLib/distanceSensor.h"
#include "TroLib/timer.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//conveyor mIntake(ports_const::intake);
TroLib::elevator mElevator(ports_const::left_elevator, ports_const::right_elevator, ports_const::elevator_third);
TroLib::piston mClaw('A');
TroLib::distanceSensor mDistanceSensor(ports_const::distance_sensor);

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

pros::Rotation v_rotation_sensor(-7);
pros::Rotation h_rotation_sensor(6);

lemlib::TrackingWheel vertical_tracking_wheel(&v_rotation_sensor, lemlib::Omniwheel::NEW_2, -0.3);
lemlib::TrackingWheel horizontal_tracking_wheel(&h_rotation_sensor, lemlib::Omniwheel::NEW_2, 2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
							&imu //inertia
							);


lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
						sensors // Odometry Sensors
						);


void initialize() {
	pros::lcd::initialize();

	// std::string text = other_const::quotes[controller.get_battery_level()%6];
	// displayOnLCD(text);

	//pros::lcd::set_text(4, random()%sizeof(other_const::quotes) / sizeof(other_const::quotes[0]));

	chassis.calibrate();
	chassis.setPose(0, 0, 0);
}

void disabled() {}

void competition_initialize() {}

//Movement right before MUST be asynchronous
TroLib::timer mAutoGrabTimer;
void autoGrab(float timeout){
	mAutoGrabTimer.reset();
	while(!mAutoGrabTimer.time_has_passed(timeout)){
		pros::delay(10);
		if(mDistanceSensor.getDistance() < claw_const::tolerance){
			mClaw.activate(false);
			chassis.cancelMotion();
			pros::delay(100);
			break;
		}
	}
}

TroLib::timer mTeleopGrabTimer;
bool mGrabEnded = false;
void teleopGrab(float timeout){
	if(!mGrabEnded){
		if(mDistanceSensor.getDistance() < claw_const::tolerance){
			mClaw.activate(false);
			mGrabEnded = true;
		}
		if(mTeleopGrabTimer.time_has_passed(timeout)){
			mGrabEnded = true;
		}
	}
}

void teleopGrabTask(void* param){
	float timeout = 1000;
	if(!mGrabEnded){
		if(mDistanceSensor.getDistance() < claw_const::tolerance){
			mClaw.activate(false);
			mGrabEnded = true;
		}
		if(mTeleopGrabTimer.time_has_passed(timeout)){
			mGrabEnded = true;
		}
	}
}

enum type{
	Match,
	Skills,
	Test
};

enum matchType{
	Far,
	Close
};

// ----- ----- -----
// CHECK EVERY TIME 
// ----- ----- -----
type mType = Test;
matchType mMatchType = Close;

void runSkill() {
	
}

void elevator_task_fn(void* param){
	while(true){
		mElevator.update();
		pros::delay(10);
	}
}

void auto_grab(void* param){
	while(true){
		if(mDistanceSensor.getDistance() < claw_const::tolerance){
			mClaw.activate(false);
			break;
		}
		pros::delay(10);
	}
}

void autonomous() {
	if(mType == Skills){
		runSkill();
	}
	if(mType == Match){
		int n = 0;
		if(mMatchType == Far){
			n = -1;
		}
		if(mMatchType == Close){
			n = 1;
		}
		//pros::Task elevator_task(elevator_task_fn, nullptr, "Elevator Task");

		mElevator.goToSetpoint(elevator_const::two);

		//First Pin
		chassis.moveToPoint(0, 2, 200, {.maxSpeed = 127}, true);
		chassis.moveToPoint(15*n, 12, 1000, {.maxSpeed = 127}, true);
		pros::delay(800);
		mElevator.goToSetpoint(elevator_const::grab);
		pros::delay(800);
		mClaw.activate(true);
		pros::delay(200);
		chassis.moveToPoint(-5*n, 14, 600, {.forwards = false, .maxSpeed = 127}, false);

		//Second Pin
		if(mMatchType == Far){
			chassis.turnToHeading(360-45, 700, {.maxSpeed = 50}, false);
		}
		if(mMatchType == Close){
			chassis.turnToHeading(45, 700, {.maxSpeed = 50}, false);
		}
		chassis.moveToPoint(9*n, 33, 1000, {.maxSpeed = 80}, false);
		autoGrab(1000);
		//mClaw.activate(false);
		pros::delay(200);
		mElevator.goToSetpoint(elevator_const::three);
		if(mMatchType == Far){
			chassis.turnToHeading(360-180, 700, {.maxSpeed = 50}, false);
		}
		if(mMatchType == Close){
			chassis.turnToHeading(180, 700, {.maxSpeed = 50}, false);
		}
		chassis.moveToPoint(24*n, 14, 1500, {.maxSpeed = 80}, true);
		pros::delay(400);
		mElevator.goToSetpoint(elevator_const::grab);
		pros::delay(400);
		mClaw.activate(true);

		//Toggle
		chassis.moveToPoint(-10*n, 28, 1000, {.forwards = false, .maxSpeed = 127}, false);
		chassis.turnToHeading(0, 700, {.maxSpeed = 50}, false);
		chassis.tank(-127, -127);
		pros::delay(1200);
		chassis.tank(127, 127);
		pros::delay(400);
		chassis.tank(-127, -127);
		pros::delay(800);
		chassis.tank(127, 127);
		pros::delay(200);
		chassis.tank(0, 0);
		mElevator.goToSetpoint(elevator_const::grab);
		// chassis.moveToPoint(0, -15, 1000, {.forwards = false, .minSpeed = 100}, false);
		// pros::delay(500);
		// chassis.moveToPoint(0, 10, 1000, {.forwards = true, .minSpeed = 100}, false);
		// pros::delay(500);
		// chassis.moveToPoint(0, -25, 1000, {.forwards = false, .minSpeed = 100}, false);

		//Third Pin
		// pros::delay(100);
		// //chassis.moveToPoint(24*n, 30, 1500, {.forwards = false, .maxSpeed = 127}, false);
		// if(mMatchType == Far){
		// 	chassis.turnToHeading(360-135, 700, {.maxSpeed = 50}, false);
		// }
		// if(mMatchType == Close){
		// 	chassis.turnToHeading(135, 700, {.maxSpeed = 50}, false);
		// }
		// chassis.moveToPoint(41*n, 15, 1500, {.maxSpeed = 80}, false);
		// chassis.moveToPoint(47*n, 9, 600, {.maxSpeed = 50}, true);
		// //autoGrab(600);
		// pros::delay(600);
		// mClaw.activate(false);
		// pros::delay(200);
		// mElevator.goToSetpoint(elevator_const::four);
		// if(mMatchType == Far){
		// 	chassis.turnToHeading(360-270, 700, {.maxSpeed = 50}, false);
		// }
		// if(mMatchType == Close){
		// 	chassis.turnToHeading(270, 700, {.maxSpeed = 50}, false);
		// }
		// chassis.moveToPoint(28*n, 4, 1500, {.maxSpeed = 127}, true);
		// pros::delay(600);
		// mElevator.goToSetpoint(elevator_const::grab);
		// pros::delay(200);
		// mClaw.activate(true);
		// pros::delay(100);

		//Escape!!!
		//chassis.moveToPoint(45*n, 5, 1500, {.forwards = false, .maxSpeed = 127}, true);

		//elevator_task.remove();
	}
	if(mType == Test){
		//chassis.turnToHeading(270, 4000);
		chassis.moveToPoint(-10, 30, 4000, {.maxSpeed = 50}, false);
		chassis.moveToPoint(-10, 15, 4000, {.forwards = false, .maxSpeed = 50}, false);
		chassis.moveToPoint(70, 70, 4000, {.maxSpeed = 50}, false);
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

	pros::Task elevator_task(elevator_task_fn, nullptr, "Elevator Task");

	while (true) {
		//pros::lcd::set_text(0, "Elevator Position: " + std::to_string(mElevator.getCurrentPosition()));
		//pros::lcd::set_text(1, "Elevator Target: " + std::to_string(mElevator.getTargetPosition()));
		//pros::lcd::set_text(2, "Elevator Setpoint: " + std::to_string(mElevator.getCurrentSetpointIndex()));
		//pros::lcd::set_text(3, "GoToPose: " + std::to_string(mDistanceSensor.getDistance()));

		//pros::lcd::print(1, "X: %f", chassis.getPose().x);
		//pros::lcd::print(2, "Y: %f", chassis.getPose().y);

		pros::lcd::print(1, "Distance: %.2f", mDistanceSensor.getDistance());
		//pros::lcd::print(2, "Distance Bool: %s", (mDistanceSensor.getDigital() ? "true" : "false"));

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
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			//Resets everything for teleopGrab
			mTeleopGrabTimer.reset();
			mGrabEnded = false;
		}
		teleopGrab(3000);		

		// Setpoint Elevator Control
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
			mElevator.incrementSetpoint(1);
		}
		else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
			mElevator.incrementSetpoint(-1);
		}
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			mElevator.goToSetpoint(elevator_const::grab);
		}

		// Manual Elevator Control
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			mElevator.run(1);
			mUpLateState = true;
		}
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			mElevator.run(-1);
			mDownLateState = true;
		}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && mUpLateState){
			mElevator.run(0);
			mUpLateState = false;
		}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && mDownLateState){
			mElevator.run(0);
			mDownLateState = false;
		}
		
		pros::delay(20);
	}

	elevator_task.remove();
}