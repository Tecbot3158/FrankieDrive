/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Wrist extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    DoubleSolenoid hatchSolenoid;

    public Wrist() {
        hatchSolenoid = new DoubleSolenoid(RobotMap.HATCH_SOLENOID_PORT[0],RobotMap.HATCH_SOLENOID_PORT[1]);

    }

    public void closeHatchSolenoid(){
        hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public void openHatchSolenoid(){
        hatchSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public DoubleSolenoid.Value getSolenoidState(){
        return this.hatchSolenoid.get();
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
