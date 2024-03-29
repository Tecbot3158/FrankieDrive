/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DefaultDrive extends Command {

    boolean hasSetAngle;
    double angle;

    public DefaultDrive() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        // left y
        double y = -(Robot.oi.getPilot().getRawAxis(1) );
        // left x
        double x = (Robot.oi.getPilot().getRawAxis(0) );
        // right x
        double turn = (Robot.oi.getPilot().getRawAxis(2));
        // Triggers
        double middleWheel = Robot.oi.getPilot().getRawAxis(3) - Robot.oi.getPilot().getRawAxis(2);

        Robot.chassis.defaultDrive(x,y,turn,middleWheel);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
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
