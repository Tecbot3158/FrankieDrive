/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

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

            double y = -OI.ground(Robot.oi.getPilot().getRawAxis(1) ,.1);
            double x = OI.ground(Robot.oi.getPilot().getRawAxis(0) ,.1);
            double turn = OI.ground(Robot.oi.getPilot().getRawAxis(4) ,.1);

        if (!Robot.chassis.isMovingMecanum() && !Robot.chassis.isMovingSwerve()) {

            Robot.chassis.drive( x, y);
            Robot.chassis.setWheel(Robot.oi.getPilot().getRawAxis(3) - Robot.oi.getPilot().getRawAxis(2));

            hasSetAngle = false;
        }else{
            if(Robot.chassis.isMovingSwerve()) Robot.chassis.swerveMove(x, y, turn);
            else Robot.chassis.mecanumDrive( x, y, turn);
            /*
            System.out.println(Robot.tecbotGyro.getYaw());
            if(!hasSetAngle){
                angle = Robot.tecbotGyro.getYaw();
                hasSetAngle = true;
            }
            System.out.println("Mecanum");
            double correction = RobotMap.turnCorrection*(Robot.tecbotGyro.getYaw() - angle);

            double middleWheel = Robot.oi.getPilot().getRawAxis(0);
            double leftSide = RobotMap.middleSidesCorrection*( -Robot.oi.getPilot().getRawAxis(1)- correction);
            double rightSide = RobotMap.middleSidesCorrection*( -Robot.oi.getPilot().getRawAxis(1) + correction);

            Robot.chassis.driveBySides(leftSide,rightSide);
            Robot.chassis.setWheel(middleWheel);
            */
        }
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
