/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class QuickTurn extends Command {


    boolean onTarget = false;
    double initialAngle;
    double targetAngle;
    public QuickTurn() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        System.out.println("Hiiii");
        Robot.chassis.setWheelState(false);
        onTarget = false;
        initialAngle = Robot.tecbotGyro.getYaw();
        if(initialAngle >=0)
            targetAngle = initialAngle -180;
        else
            targetAngle = initialAngle +180;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double deltaAngle = Math.abs(-Robot.tecbotGyro.getYaw() + targetAngle);

        Robot.chassis.driveBySides(-deltaAngle * RobotMap.QUICK_TURN_CORRECTION, deltaAngle * RobotMap.QUICK_TURN_CORRECTION);
        if(Math.abs(deltaAngle) <= RobotMap.TURN_OFFSET){
            onTarget = true;
        }
        System.out.println(deltaAngle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return onTarget;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.chassis.setWheelState(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
