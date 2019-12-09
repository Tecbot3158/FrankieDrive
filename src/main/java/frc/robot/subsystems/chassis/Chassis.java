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
import frc.robot.RobotMap;
import frc.robot.TecbotSpeedController;
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
        } else{
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
        //left.set(0.4);
        //right.set(-0.4);

        SmartDashboard.putNumber("leftS", leftPower);
        SmartDashboard.putNumber("rightS", rightPower);
        SmartDashboard.putNumber("turn", turn);
        SmartDashboard.putNumber("speed", speed);


    }

    public void driveBySides(double leftPower, double rightPower){

        leftRear.set(leftPower);
        leftFront.set(leftPower);

        rightFront.set(-rightPower);
        rightRear.set(rightPower);

    }

    public void setWheel(double value) {

        middle.set(value);

    }


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new DefaultDrive());
    }
}
