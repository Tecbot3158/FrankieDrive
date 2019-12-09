/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SwerveMove extends Command {

    double startingAngle;

    public SwerveMove() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        startingAngle = Robot.tecbotGyro.getYaw();
        System.out.println("Swerve");

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        System.out.println(Robot.tecbotGyro.getYaw());
        double correction = RobotMap.turnCorrection*(Robot.tecbotGyro.getYaw() - startingAngle);

        double middleWheel = Robot.oi.getPilot().getRawAxis(0);
        double leftSide = RobotMap.middleSidesCorrection*( Robot.oi.getPilot().getRawAxis(1) - correction);
        double rightSide = RobotMap.middleSidesCorrection*( Robot.oi.getPilot().getRawAxis(1) + correction);

        Robot.chassis.driveBySides(leftSide,rightSide);
        Robot.chassis.setWheel(middleWheel);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return !Robot.movingSwerve;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
