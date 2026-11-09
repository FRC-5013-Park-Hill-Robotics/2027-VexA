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
#include "TroLib/claw.h"
#include "TroLib/averageOverTime.h"

float acceleration = 0;
float lastKnownVelocity = 0;

float time1 = 0;
float distance1 = 0;
float time2 = 0;
float distance2 = 0;

float interpolator(float interpolateTime)
{
    if (time2 == time1)
        return 1000;

    // Calculate the rate of change per unit of time.
	float dt = time1 - time2;

    float currentVelocity = (distance1 - distance2) /
                  (dt);
	acceleration = (currentVelocity - lastKnownVelocity) / (dt);
	
	lastKnownVelocity = currentVelocity;

    // Project the trend to the requested future time.
	float projectedTime = interpolateTime;

	// Use the kinematic equation: s = s0 + v0*t + 0.5*a*t^2
    return distance1 + currentVelocity * (projectedTime) + 0.5 * acceleration * (projectedTime * projectedTime);
}

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//conveyor mIntake(ports_const::intake);
TroLib::elevator mElevator(ports_const::left_elevator, ports_const::right_elevator, ports_const::elevator_third);
TroLib::claw mClaw(ports_const::distance_sensor, 'A');

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

void disabled() {
	mElevator.run(0);
	mClaw.activate(false);
}

void competition_initialize() {}

enum type{
	Match,
	Skills,
	Test
};

enum matchType{
	Far,
	Close,
	Far2pin,
	Close2pin
};

// ----- ----- -----
// CHECK EVERY TIME 
// ----- ----- -----
type mType = Match;
matchType mMatchType = Far2pin;

void elevator_task_fn(void* param){
	while(true){
		mElevator.update();
		pros::delay(10);
	}
}

void claw_task_fn(void* param){
	while(true){
		mClaw.update();
		pros::delay(3);
	}
}


void runSkill() {
	// int n = 0;
	// if(mMatchType == Far){
	// 	n = -1;
	// }
	// if(mMatchType == Close){
	// 	n = 1;
	// }
	// pros::Task elevator_task(elevator_task_fn, nullptr, "Elevator Task");
	// pros::Task claw_task(claw_task_fn, nullptr, "Claw Task");

	// chassis.setPose(-2*n, 0, 0);

	// //Toggle
	// mElevator.toggleTwice();
	// pros::delay(500);

	// //First Pin
	// chassis.moveToPoint(0, 2, 200, {.maxSpeed = 127}, false);
	// mElevator.goToSetpoint(elevator_const::oneM);
	// chassis.moveToPoint(12.5*n, 11, 1000, {.maxSpeed = 127}, true);
	// pros::delay(800);
	// mElevator.goToSetpoint(elevator_const::grab);
	// pros::delay(600);
	// mClaw.activate(true);
	// pros::delay(100);
	// mElevator.goToSetpoint(elevator_const::two);
	// chassis.moveToPoint(-5*n, 14, 600, {.forwards = false, .maxSpeed = 127}, false);

	// //Second Pin
	// if(mMatchType == Far){
	// 	chassis.turnToHeading(360-45, 700, {.maxSpeed = 50}, false);
	// }
	// if(mMatchType == Close){
	// 	chassis.turnToHeading(45, 700, {.maxSpeed = 50}, false);
	// }
	// mClaw.autograb(2000);
	// chassis.moveToPoint(15*n, 35, 1500, {.maxSpeed = 50}, false);
	// mElevator.goToSetpoint(elevator_const::two);
	// pros::delay(200);
	// if(mMatchType == Far){
	// 	chassis.turnToHeading(360-180, 700, {.maxSpeed = 50}, false);
	// }
	// if(mMatchType == Close){
	// 	chassis.turnToHeading(180, 700, {.maxSpeed = 50}, false);
	// }
	// pros::delay(100);
	// chassis.moveToPoint(20*n, 15, 1500, {.maxSpeed = 50}, true);
	// pros::delay(900);
	// mElevator.goToSetpoint(elevator_const::grab);
	// pros::delay(300);
	// mClaw.activate(true);

	// //Third Pin
	// pros::delay(200);
	// chassis.moveToPoint(23*n, 36, 1500, {.forwards = false, .maxSpeed = 127}, false);
	// if(mMatchType == Far){
	// 	chassis.turnToHeading(360-135, 700, {.maxSpeed = 50}, false);
	// }
	// if(mMatchType == Close){
	// 	chassis.turnToHeading(135, 700, {.maxSpeed = 50}, false);
	// }
	// mClaw.autograb(2000);
	// chassis.moveToPoint(43*n, 17, 1500, {.maxSpeed = 50}, false);
	// pros::delay(500);
	// mElevator.goToSetpoint(elevator_const::three);
	// if(mMatchType == Far){
	// 	chassis.turnToHeading(360-270, 700, {.maxSpeed = 50}, false);
	// }
	// if(mMatchType == Close){
	// 	chassis.turnToHeading(270, 700, {.maxSpeed = 50}, false);
	// }
	// chassis.moveToPoint(21*n, 13, 1500, {.maxSpeed = 40}, false);
	// mElevator.goToSetpoint(elevator_const::one);
	// pros::delay(400);
	// mElevator.run(0);
	// mClaw.activate(true);
	// pros::delay(100);

	// //Escape!!!
	// chassis.moveToPoint(50*n, 5, 200, {.forwards = false, .maxSpeed = 127}, false); 
	// chassis.moveToPoint(5*n, 55, 2000, {.forwards = false, .maxSpeed = 80}, false);

	// elevator_task.remove();
	// claw_task.remove();
}

void autonomous() {
	if(mType == Skills){
		runSkill();
	}
	if(mType == Match){
		int n = 0;
		if(mMatchType == Far || mMatchType == Far2pin){
			n = -1;
		}
		if(mMatchType == Close || mMatchType == Close2pin){
			n = 1;
		}
		pros::Task elevator_task(elevator_task_fn, nullptr, "Elevator Task");
		pros::Task claw_task(claw_task_fn, nullptr, "Claw Task");

		chassis.setPose(-2*n, 0, 0);

		//Toggle
		mElevator.toggleTwice();
		pros::delay(500);

		//First Pin
		chassis.moveToPoint(0, 2, 200, {.maxSpeed = 127}, false);
		mElevator.goToSetpoint(elevator_const::oneM);
		chassis.moveToPoint(12.5*n, 11, 1000, {.maxSpeed = 127}, true);
		pros::delay(800);
		mElevator.goToSetpoint(elevator_const::grab);
		pros::delay(600);
		mClaw.activate(true);
		pros::delay(100);
		mElevator.goToSetpoint(elevator_const::two);
		chassis.moveToPoint(-5*n, 14, 600, {.forwards = false, .maxSpeed = 127}, false);

		if(mMatchType == Close || mMatchType == Far){
			//Second Pin
			if(mMatchType == Far){
				chassis.turnToHeading(360-45, 700, {.maxSpeed = 50}, false);
			}
			if(mMatchType == Close){
				chassis.turnToHeading(45, 700, {.maxSpeed = 50}, false);
			}
			mClaw.autograb(2000);
			chassis.moveToPoint(15*n, 35, 1500, {.maxSpeed = 50}, false);
			mElevator.goToSetpoint(elevator_const::two);
			pros::delay(200);
			if(mMatchType == Far){
				chassis.turnToHeading(360-180, 700, {.maxSpeed = 50}, false);
			}
			if(mMatchType == Close){
				chassis.turnToHeading(180, 700, {.maxSpeed = 50}, false);
			}
			pros::delay(100);
			chassis.moveToPoint(20*n, 15, 1500, {.maxSpeed = 50}, true);
			pros::delay(900);
			mElevator.goToSetpoint(elevator_const::grab);
			pros::delay(300);
			mClaw.activate(true);

			//Third Pin
			pros::delay(200);
			chassis.moveToPoint(23*n, 36, 1500, {.forwards = false, .maxSpeed = 127}, false);
			if(mMatchType == Far){
				chassis.turnToHeading(360-135, 700, {.maxSpeed = 50}, false);
			}
			if(mMatchType == Close){
				chassis.turnToHeading(135, 700, {.maxSpeed = 50}, false);
			}
			mClaw.autograb(2000);
			chassis.moveToPoint(43*n, 17, 1500, {.maxSpeed = 50}, false);
			pros::delay(500);
			mElevator.goToSetpoint(elevator_const::three);
			if(mMatchType == Far){
				chassis.turnToHeading(360-270, 700, {.maxSpeed = 50}, false);
			}
			if(mMatchType == Close){
				chassis.turnToHeading(270, 700, {.maxSpeed = 50}, false);
			}
			chassis.moveToPoint(21*n, 13, 1500, {.maxSpeed = 40}, false);
			mElevator.goToSetpoint(elevator_const::one);
			pros::delay(400);
			mElevator.run(0);
			mClaw.activate(true);
			pros::delay(100);

			//Escape!!!
			chassis.moveToPoint(45*n, 5, 200, {.forwards = false, .maxSpeed = 127}, true);
		}	
		if(mMatchType == Close2pin || mMatchType == Far2pin){
			mElevator.goToSetpoint(elevator_const::grab);
			if(mMatchType == Far){
				chassis.turnToHeading(360-135, 700, {.maxSpeed = 50}, false);
			}
			if(mMatchType == Close){
				chassis.turnToHeading(135, 700, {.maxSpeed = 50}, false);
			}
			chassis.moveToPoint(9.5*n, 5, 1000, {.maxSpeed = 50}, false);
			chassis.moveToPoint(10*n, 0, 1000, {.maxSpeed = 50}, false);
			chassis.moveToPoint(11.5*n, -5, 1000, {.maxSpeed = 40}, false);
			chassis.moveToPoint(11*n, -8, 1000, {.maxSpeed = 80}, false);
			mClaw.activate(false);
			pros::delay(200);
			mElevator.goToSetpoint(elevator_const::two);
			if(mMatchType == Far){
				chassis.turnToHeading(360-90, 700, {.maxSpeed = 50}, false);
			}
			if(mMatchType == Close){
				chassis.turnToHeading(90, 700, {.maxSpeed = 50}, false);
			}
			chassis.moveToPoint(11.5*n, 5, 1000, {.forwards = false, .maxSpeed = 80}, false);
			chassis.moveToPoint(-5*n, 17, 1500, {.forwards = false, .maxSpeed = 80}, false);
			if(mMatchType == Far){
				chassis.turnToHeading(360-90, 700, {.maxSpeed = 50}, false);
			}
			if(mMatchType == Close){
				chassis.turnToHeading(90, 700, {.maxSpeed = 50}, false);
			}
			
			//place
			chassis.moveToPoint(12.5*n, 14, 1000, {.maxSpeed = 80}, true);
			pros::delay(800);
			mElevator.goToSetpoint(elevator_const::grab);
			pros::delay(600);
			mClaw.activate(true);
			pros::delay(100);
			//mElevator.goToSetpoint(elevator_const::two);
			chassis.moveToPoint(-5*n, 14, 600, {.forwards = false, .maxSpeed = 127}, false);	
		}
 
		elevator_task.remove();
		claw_task.remove();
	}
	if(mType == Test){
		pros::Task elevator_task(elevator_task_fn, nullptr, "Elevator Task");
		pros::Task claw_task(claw_task_fn, nullptr, "Claw Task");
		
		mElevator.toggleTwice();
		pros::delay(4000);
		// mClaw.activate(true);
		// pros::delay(500);
		// mClaw.autograb(2000);
		// chassis.moveToPoint(0, 30, 4000, {.maxSpeed = 50}, true);
		
		elevator_task.remove();
		claw_task.remove();
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
	if(mType == Skills){
		runSkill();
	}

	bool mUpLateState = false;
	bool mDownLateState = false;
	bool mToggleLateState = false;

	pros::Task elevator_task(elevator_task_fn, nullptr, "Elevator Task");
	pros::Task claw_task(claw_task_fn, nullptr, "Claw Task");

	while (true) {
		//pros::lcd::set_text(0, "Get Distance: " + std::to_string(interpolator(claw_const::interpolation_window)));
		//pros::lcd::set_text(1, "Get Interpolation Distance: " + std::to_string(mClaw.getInterpolationDist()));
		//pros::lcd::set_text(2, "Get Mode: " + mClaw.getMode());
		//pros::lcd::set_text(3, "GoToPose: " + std::to_string(mDistanceSensor.getDistance()));

		//pros::lcd::print(1, "X: %f", chassis.getPose().x);
		//pros::lcd::print(2, "Y: %f", chassis.getPose().y);

		//pros::lcd::print(1, "Distance: %.2f", mDistanceSensor.getDistance());
		//pros::lcd::print(2, "Distance Bool: %s", (mDistanceSensor.getDigital() ? "true" : "false"));

		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

		// Claw Control
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			mClaw.toggle();
		}
		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
		// 	mClaw.autograb(2000);
		// }	

		// Setpoint Elevator Control
		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
		// 	mElevator.incrementSetpoint(1);
		// }
		// else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
		// 	mElevator.incrementSetpoint(-1);
		// }
		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
		// 	mElevator.goToSetpoint(elevator_const::grab);
		// }

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
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
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			mElevator.toggle();
			mToggleLateState = true;
		}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && mUpLateState){
			mElevator.run(0);
			mUpLateState = false;
		}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && mDownLateState){
			mElevator.run(0);
			mDownLateState = false;
		}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && mToggleLateState){
			mElevator.run(0);
			mToggleLateState = false;
		}
		
		pros::delay(20);
	}

	elevator_task.remove();
	claw_task.remove();
}