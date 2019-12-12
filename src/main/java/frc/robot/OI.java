/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.chassis.LowerWheel;
import frc.robot.commands.chassis.RiseWheel;
import frc.robot.commands.chassis.TogglePivoting;
import frc.robot.commands.chassis.ToggleSwerve;
import frc.robot.subsystems.chassis.ToggleMecanum;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

    Joystick pilot;
    JoystickButton a,b,lb, rb, ls, rs;

    public OI(){

        pilot = new Joystick(0);
        a = new JoystickButton(pilot, 1);
        b = new JoystickButton(pilot, 2);
        lb = new JoystickButton(pilot, 5);
        rb = new JoystickButton(pilot, 6);
        rs = new JoystickButton(pilot, 10);
        ls = new JoystickButton(pilot, 9);

        a.whenPressed(new LowerWheel());
        b.whenPressed(new RiseWheel());
        ls.whenPressed(new ToggleMecanum());
        rs.whenPressed(new ToggleSwerve());
        rb.whenPressed(new TogglePivoting());
        rb.whenReleased(new TogglePivoting());
        lb.whenPressed(new ResetGyro());

    }
    public Joystick getPilot(){

        return pilot;
    }

    /**
     *
     * @param value The value to be grounded.
     * @param min The minimum value that it needs to have so that it is not considered 0.
     * @return Returns 0 when the absolute value is less that the minimum.
     */
    public static double ground(double value, double min){
        value = (value



                >= -min && value <= min) ? 0 : value;
        return value;
}



}
