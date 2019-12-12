/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 *
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;

    public static final double turnCorrection = .05;
    //The sides move at a different speed than the middle wheel, so this constant contols that difference to try to
    //Make them move at the same speed;
    public static final double middleSidesCorrection = .6;


    public static final int MIDDLE_WHEEL_MOTOR = 0; //TALON
    public static final TecbotSpeedController.TypeOfMotor middleWheelMotorType = TecbotSpeedController.TypeOfMotor.PWM_TALON_SRX;
    public static final int[] WHEEL_SOLENOID = {1,2};

    public static final int[] LEFT_CHASSIS_MOTORS = {6, 8}; //talon
    public static final TecbotSpeedController.TypeOfMotor leftWheelMotorType = TecbotSpeedController.TypeOfMotor.PWM_TALON_SRX;


    public static final int[] RIGHT_CHASSIS_MOTORS = {3, 5}; //talon
    public static final TecbotSpeedController.TypeOfMotor rightWheelMotorType = TecbotSpeedController.TypeOfMotor.PWM_TALON_SRX;
}
