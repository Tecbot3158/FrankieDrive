/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.chassis;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotSpeedController;
import frc.robot.commands.chassis.DefaultDrive;

/**
 * Add your docs here.
 */
public class Chassis extends Subsystem {

    TecbotSpeedController leftRear, leftFront, rightRear, rightFront;
    TecbotSpeedController middle;
    DoubleSolenoid wheelDoubleSolenoid;


    // Mecanum and Swerve move require the robot to stay in the same angle (unless turning) so hasSetAngle
    // checks if the angle has been set.
    boolean hasSetAngle;
    // The angle the robot will stay in during mecanum drive unless turning.
    double startingAngle;


    public enum DrivingMode {Default, Pivot,  Mecanum, Swerve};
    public DrivingMode currentDrivingMode;



    public Chassis() {

        leftRear = new TecbotSpeedController(RobotMap.LEFT_CHASSIS_MOTORS[0], RobotMap.leftWheelMotorType);
        leftFront = new TecbotSpeedController(RobotMap.LEFT_CHASSIS_MOTORS[1], RobotMap.leftWheelMotorType);

        rightRear = new TecbotSpeedController(RobotMap.RIGHT_CHASSIS_MOTORS[0], RobotMap.rightWheelMotorType);
        rightFront = new TecbotSpeedController(RobotMap.RIGHT_CHASSIS_MOTORS[1], RobotMap.rightWheelMotorType);

        middle = new TecbotSpeedController(RobotMap.MIDDLE_WHEEL_MOTOR, RobotMap.middleWheelMotorType);

        //wheelSolenoid = new Solenoid(7,1);
        wheelDoubleSolenoid = new DoubleSolenoid(RobotMap.WHEEL_SOLENOID[0], RobotMap.WHEEL_SOLENOID[1]);

    }

    /**
     * The default driving method for all driving modes.
     * @param x The value of the x axis
     * @param y The value of the y axis
     * @param turn The value of the axis designated for turing
     * @param middleWheel The value that will be given to the middle wheel
     */
    public void defaultDrive(double x, double y, double turn, double middleWheel){

        switch (currentDrivingMode){
            case Default:
                frankieTankDrive(x,y, middleWheel);
                break;
            case Pivot:
                pivot(x,y);
                break;
            case Mecanum:
                mecanumDrive(x,y,turn);
                break;
            case Swerve:
                swerveMove(x, y, turn);
                break;
            default:
                DriverStation.reportError("Driving mode not recognized", true);
        }

    }

    /**
     * Rises or lowers the wheel.
     * @param state The desired state for the wheel, true for rising.
     */

    public void setWheelState(boolean state) {
        //wheelSolenoid.set(state);
        if (state) {
            wheelDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            wheelDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        SmartDashboard.putString("solenoid1", wheelDoubleSolenoid.get().toString());

    }

    /**
     * The default driving method for driving a frankie-type chassis as a tank
     * @param turn  The value of the joystick used for turning.
     * @param speed The value of the joystick used for moving straight.
     * @param middleWheel The value that will be given to the middle wheel
     */
    public void frankieTankDrive(double turn, double speed, double middleWheel) {
        double leftPower = (turn + speed);
        double rightPower = -turn + speed;

        leftRear.set(leftPower);
        leftFront.set(leftPower);

        rightFront.set(-rightPower);
        rightRear.set(rightPower);

        setWheel(middleWheel);

    }

    /**
     * Moves the robot pivoting in left or right wheels
     * @param turn The value of the joystick used for turning.
     * @param speed The value of the joystick used for moving straight.
     */
    public  void pivot(double turn, double speed){
        if (turn <= 0) {
            Robot.chassis.driveBySides(-.1, speed);
        } else {
            Robot.chassis.driveBySides(speed, -.1);
        }
    }

    /**
     * Controls the robot sides independently.
     *
     * @param leftPower  The amount of power that will be given to all the motors in the left side.
     * @param rightPower The amount of power that will be given to all the motors in the right side.
     */
    public void driveBySides(double leftPower, double rightPower) {

        leftRear.set(leftPower);
        leftFront.set(leftPower);

        rightFront.set(-rightPower);
        rightRear.set(rightPower);

    }

    public void setWheel(double value) {

        middle.set(value);

    }

    /**
     * This method controls the robot as if it were a mecanum chassis.
     * <br><strong>No field orientated drive is implemented in this method.</strong>
     *
     * @param x    The desired movement in x axis, from -1 to 1.
     * @param y    The desired movement in the y axis, from -1 to 1.
     * @param turn The desired turn that the robot will have while driving, from -1 to 1.
     */
    public void mecanumDrive(double x, double y, double turn) {

        if (!hasSetAngle) {
            startingAngle = Robot.tecbotGyro.getYaw();
            hasSetAngle = true;

            // This condition will happen once every time the robot enters mecanum drive.
            // Mecanum drive needs to be lowered. We need to lower the wheel once the robot
            // enters mecanum drive.
            setWheelState(false);
        }
        if (turn >= .1 || turn <= -.1) startingAngle = Robot.tecbotGyro.getYaw();

        double deltaAngle = Robot.tecbotGyro.getYaw() - startingAngle;
        System.out.println("f"+deltaAngle);

        //Prevents robot from turning in the incorrect direction
        if(deltaAngle > 180) {
            deltaAngle = deltaAngle - 360;
        }
        else if(deltaAngle < -180) {
            deltaAngle = -deltaAngle + 360;
        }
        System.out.println(deltaAngle);
        double correction = RobotMap.TURN_CORRECTION * deltaAngle;

        double leftSide = RobotMap.MIDDLE_SIDES_CORRECTION * (y - correction + turn);
        double rightSide = RobotMap.MIDDLE_SIDES_CORRECTION * (y + correction - turn);

        Robot.chassis.driveBySides(leftSide, rightSide);
        Robot.chassis.setWheel(x);

    }


    /**
     * This method takes an angle and makes the robot move in that direction using the middle wheel.
     * It can also turn the robot while moving.
     * <br><strong>No field orientated drive is implemented in this method.</strong>
     *
     * @param angle    The angle in degrees relative to the robot at which the robot will move.
     * @param maxPower The max power that will be given to the motors.
     * @param turn     The desired turn that the robot will have while driving, from -1 to 1.
     */
    public void driveToAngle(double angle, double maxPower, double turn) {
        double x = Math.sin(Math.toRadians(angle)) * maxPower;
        double y = Math.cos(Math.toRadians(angle)) * maxPower;

        mecanumDrive(x, y, turn);
    }

    /**
     * This method uses field orientated drive to make the robot move a certain value in x and a
     * certain value in y while turning.
     *
     * @param x    The desired movement that the robot will have in the x axis
     * @param y    The desired movement that the robot will have in the y axis
     * @param turn The desired turn that the robot will have while driving, from -1 to 1.
     */
    public void swerveMove(double x, double y, double turn) {
        // The angle relative to the field given by the x and the y
        double absoluteAngle = 0;
        if (y != 0)
            absoluteAngle = Math.toDegrees(Math.atan(x / y));
        if (y < 0) {
            if (x < 0) {
                absoluteAngle -= 180;
            } else {
                absoluteAngle += 180;
            }
        }
        // The angle at which the robot will move, considering its rotation.
        double relativeAngle = absoluteAngle - Robot.tecbotGyro.getYaw();
        // The max power that will be given to the motors.
        double speed = Math.sqrt((x * x) + (y * y));


        driveToAngle(relativeAngle, speed, turn);

    }



    public void setMecanumDrive(boolean state) {
       if(state) currentDrivingMode = DrivingMode.Mecanum;
       else currentDrivingMode = DrivingMode.Default;
    }

    public boolean isMovingMecanum() {
        return (currentDrivingMode == DrivingMode.Mecanum);
    }

    public void setSwerveDrive(boolean state) {
        if(state) currentDrivingMode = DrivingMode.Swerve;
        else currentDrivingMode = DrivingMode.Default;
    }

    public boolean isMovingSwerve() {
        return (currentDrivingMode == DrivingMode.Swerve);
    }

    public void setPivoting(boolean state) {
        if(state) currentDrivingMode = DrivingMode.Pivot;
        else currentDrivingMode = DrivingMode.Default;
    }

    public boolean isPivoting() {
        return (currentDrivingMode == DrivingMode.Pivot);
    }


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new DefaultDrive());
    }
}
