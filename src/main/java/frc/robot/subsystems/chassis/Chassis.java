/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.chassis;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    TecbotSpeedController leftRear, leftFront, rightRear, rightFront;
    TecbotSpeedController middle;
    Solenoid wheelSolenoid, solenoid2, solenoid3, solenoid4;
    DoubleSolenoid wheelDoubleSolenoid;

    // Making movingMecanum true will make the robot change its drive to a mecanum like.
    // Mecanum move requires the robot to stay in the same angle (unless turning) so has set angle
    // checks if the angle has been set.
    boolean movingMecanum = false;
    boolean movingSwerve = false;
    boolean hasSetAngle;

    boolean isPivoting = false;

    public boolean isPivoting() {
        return isPivoting;
    }

    public void setPivoting(boolean pivoting) {
        isPivoting = pivoting;
    }

    // The angle the robot will stay in during mecanum drive unless turning.
    double startingAngle;

    public Chassis() {

        leftRear = new TecbotSpeedController(RobotMap.LEFT_CHASSIS_MOTORS[0], RobotMap.leftWheelMotorType);
        leftFront = new TecbotSpeedController(RobotMap.LEFT_CHASSIS_MOTORS[1], RobotMap.leftWheelMotorType);

        rightRear = new TecbotSpeedController(RobotMap.RIGHT_CHASSIS_MOTORS[0], RobotMap.rightWheelMotorType);
        rightFront = new TecbotSpeedController(RobotMap.RIGHT_CHASSIS_MOTORS[1], RobotMap.rightWheelMotorType);

        middle = new TecbotSpeedController(RobotMap.MIDDLE_WHEEL_MOTOR, RobotMap.middleWheelMotorType);

        //wheelSolenoid = new Solenoid(7,1);
        wheelDoubleSolenoid = new DoubleSolenoid(0, 1);

    }

    public void setWheelState(boolean state) {

        //wheelSolenoid.set(state);
        if (state) {
            wheelDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            wheelDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        SmartDashboard.putString("solenoid1", wheelDoubleSolenoid.get().toString());

    }

    public void drive(double turn, double speed) {
        double leftPower = (turn + speed);
        double rightPower = -turn + speed;

        leftRear.set(leftPower);
        leftFront.set(leftPower);

        rightFront.set(-rightPower);
        rightRear.set(rightPower);

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

        double correction = RobotMap.turnCorrection * (Robot.tecbotGyro.getYaw() - startingAngle);

        double middleWheel = x;

        double leftSide = RobotMap.middleSidesCorrection * (y - correction + turn);
        double rightSide = RobotMap.middleSidesCorrection * (y + correction - turn);

        Robot.chassis.driveBySides(leftSide, rightSide);
        Robot.chassis.setWheel(middleWheel);

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
        movingMecanum = state;
        if (state) movingSwerve = false;
        if (!state) hasSetAngle = false;
    }

    public boolean isMovingMecanum() {
        return movingMecanum;
    }

    public void setSwerveDrive(boolean state) {
        movingSwerve = state;
        if (state) movingMecanum = false;
        if (!state) hasSetAngle = false;
    }

    public boolean isMovingSwerve() {
        return movingSwerve;
    }


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new DefaultDrive());
    }
}
