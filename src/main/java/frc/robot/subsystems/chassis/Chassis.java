/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.chassis;

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

    TecbotSpeedController[] left = new TecbotSpeedController[2], right = new TecbotSpeedController[2];
    TecbotSpeedController middle;
    Solenoid wheelSolenoid;

    public Chassis() {

        left = new TecbotSpeedController[]{new TecbotSpeedController(RobotMap.LEFT_CHASSIS_MOTORS[0], RobotMap.leftWheelMotorType), new TecbotSpeedController(RobotMap.LEFT_CHASSIS_MOTORS[1],RobotMap.leftWheelMotorType)};
        right = new TecbotSpeedController[]{new TecbotSpeedController(RobotMap.RIGHT_CHASSIS_MOTORS[0], RobotMap.rightWheelMotorType), new TecbotSpeedController(RobotMap.RIGHT_CHASSIS_MOTORS[1], RobotMap.rightWheelMotorType)};

        middle = new TecbotSpeedController(RobotMap.MIDDLE_WHEEL_MOTOR, RobotMap.middleWheelMotorType);

        wheelSolenoid = new Solenoid(RobotMap.WHEEL_SOLENOID[0], RobotMap.WHEEL_SOLENOID[1]);

    }

    public void setWheelState(boolean state) {

        wheelSolenoid.set(state);

    }

    public void drive(double turn, double speed) {
        double leftPower = turn + speed;
        double rightPower = -turn + speed;
        for (TecbotSpeedController a : left) {
            a.set(leftPower*.25);
        }
        for (TecbotSpeedController a : right) {
            a.set(rightPower);
        }
        //left.set(0.4);
        //right.set(-0.4);

        SmartDashboard.putNumber("leftS", leftPower);
        SmartDashboard.putNumber("rightS",rightPower);
        SmartDashboard.putNumber("turn",turn);
        SmartDashboard.putNumber("speed",speed);


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
