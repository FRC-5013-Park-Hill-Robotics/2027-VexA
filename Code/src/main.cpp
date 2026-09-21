#include "main.h"

#include "lemlib/api.hpp"

#include "conveyor.h"
#include "piston.h"
#include "constants.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

conveyor mTunnel(ports_const::tunnel);

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
pros::MotorGroup left_mg({-7, -18, 19}, drive_const::gearset);    // Creates a motor group 
pros::MotorGroup right_mg({9, -10, 20}, drive_const::gearset);  // + for forwards, - for reversed

lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.5, // track width (in)
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              300, //Drivetrain rpm
							  2 //drift
							  );

pros::Imu imu(ports_const::imu);

//pros::Rotation h_rotation_sensor(5);
pros::Rotation v_rotation_sensor(4);

//lemlib::TrackingWheel horizontal_tracking_wheel(&h_rotation_sensor, lemlib::Omniwheel::NEW_2, 3.5);
lemlib::TrackingWheel vertical_tracking_wheel(&v_rotation_sensor, lemlib::Omniwheel::NEW_2, 0);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel 1
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
	pros::lcd::set_text(1, "The Trobots Presents...\n	THE BOT");
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
void autonomous() {}

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
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);
		
		pros::delay(20);
	}
}