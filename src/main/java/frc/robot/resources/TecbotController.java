package frc.robot.resources;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

import java.util.*;

/**
 * A Tecbot Controller is a Joystick controller in which you can get different values from your controller,
 * whether it is a Ps4, Ps3, Xbox ONE, Xbox 360 or any other controller.
 */
public class TecbotController {


    /**
     * The ports for the axis on the PS4 Controller in the following order:
     * <ul>
     *     <li>Left Axis X</li>
     *     <li>Left Axis Y</li>
     *     <li>Right Axis X</li>
     *     <li>Right Axis Y</li>
     * </ul>
     */
    private int[] portsJoysticksPS4 = {0, 1, 2, 5};

    /**
     * The ports for the axis on the XBOX Controller in the following order:
     * <ul>
     *     <li>Left Axis X</li>
     *     <li>Left Axis Y</li>
     *     <li>Right Axis X</li>
     *     <li>Right Axis Y</li>
     * </ul>
     */
    private int[] portsJoystickXBOX = {0, 1, 4, 5};

    /**
     * The ports for the buttons in PS4 controller.
     * <br>
     * In the following order:
     * <strong>
     * <ul>
     *     <li>a</li>
     *     <li>b</li>
     *     <li>x</li>
     *     <li>y</li>
     *     <li>lb</li>
     *     <li>rb</li>
     *     <li><i>BACK</i></li>
     *     <li><i>START</i></li>
     *     <li>LS</li>
     *     <li>RS</li>
     * </ul>
     * </strong>
     */
    private int[] portsButtonsPS4 = {2, 3, 1, 4, 5, 6, 9, 10, 11, 12};
    /**
     * The ports for the buttons in xbox controller.
     * <br>
     * In the following order:
     * <strong>
     * <ul>
     *     <li>a</li>
     *     <li>b</li>
     *     <li>x</li>
     *     <li>y</li>
     *     <li>lb</li>
     *     <li>rb</li>
     *     <li><i>BACK</i></li>
     *     <li><i>START</i></li>
     *     <li>LS</li>
     *     <li>RS</li>
     * </ul>
     * </strong>
     */
    private int[] portsButtonsXBOX = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    /**
     * <h1>Trigger ports for PS4</h1>
     * <br>
     * In the following order:
     * <ul>
     *     <li>Left Trigger Value</li>
     *     <li>Right Trigger Value</li>
     * </ul>
     */
    private int[] portsTriggersPS4 = {3, 4};
    /**
     * <h1>Trigger ports for XBOX</h1>
     * <br>
     * In the following order:
     * <ul>
     *     <li>Left Trigger Value</li>
     *     <li>Right Trigger Value</li>
     * </ul>
     */
    private int[] portsTriggersXBOX = {2, 3};

    /**
     * Joystick object which is used to get information from axis and used as a
     * reference for buttons.
     */
    private Joystick pilot;


    TypeOfController controllerType;

    /**
     * This array contains all the buttons used to set properties such as
     * <i>whenPressed(), whenReleased(), whileHeld(). </i>
     */
    private JoystickButton[] buttons;

    /**
     * The offset that will correct for Joystick values.
     */
    private double offset = 0.1;

    private enum TypeOfController {
        PS4,
        XBOX
    }

    public enum ButtonType {
        A,
        B,
        X,
        Y,
        LB,
        RB,
        BACK,
        START,
        LS,
        RS
    }

    /**
     * @param port The port that the controller has in the Driver Station.
     */
    public TecbotController(int port) {
        pilot = new Joystick(port);

        controllerType = null;
        if (pilot.getName().toLowerCase().contains("wireless controller")) controllerType = TypeOfController.PS4;
        if (pilot.getName().toLowerCase().contains("xbox")) controllerType = TypeOfController.XBOX;

        if (pilot.getName() == null) DriverStation.reportWarning("Joystick not found (Tecbot Controller)", false);
        if (controllerType != null) setButtons();
        else DriverStation.reportWarning("Controller not identified, some methods will return 0.", false);

    }

    /**
     * If your controller is not recognized as PS4 or XBOX, use {@link #getRawAxis(int axis, boolean ground)} instead.
     * @return Left Joystick Axis X
     */
    public double getLeftAxisX() {
        double value;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[0]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[0]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getLeftAxisX(). Returned 0. Use getRawAxis() instead.", false);
                break;
        }
        return ground(value, getOffset());
    }

    /**
     * If your controller is not recognized as PS4 or XBOX, use {@link #getRawAxis(int axis, boolean ground)} instead.
     * @return Left Joystick Axis Y
     */
    public double getLeftAxisY() {
        double value;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[1]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[1]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getLeftAxisY(). Returned 0. Use getRawAxis() instead.", false);
                break;
        }
        return ground(value, getOffset());
    }


    /**
     * If your controller is not recognized as PS4 or XBOX, use {@link #getRawAxis(int axis, boolean ground)} instead.
     * @return Right Joystick Axis X
     */
    public double getRightAxisX() {
        double value;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[2]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[2]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getRightAxisX(). Returned 0. Use getRawAxis() instead.", false);
                break;
        }
        return ground(value, offset);
    }

    /**
     * If your controller is not recognized as PS4 or XBOX, use {@link #getRawAxis(int axis, boolean ground)} instead.
     * @return Right Joystick Axis Y
     */
    public double getRightAxisY() {
        double value;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[3]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[3]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getRightAxisY(). Returned 0. Use getRawAxis() instead.", false);
                break;
        }
        return ground(value, getOffset());
    }

    /**
     * @param axis axis port in the controller.
     * @param ground If true, it will correct according to the offset from {@link #getOffset()}
     * @return value of given axis.
     */
    public double getRawAxis(int axis, boolean ground) {
        return ground ? ground(pilot.getRawAxis(axis), offset) : pilot.getRawAxis(axis);
    }

    /**
     * Returns the value of the button.
     *
     * @param buttonNumber The button to be read.
     * @return The state of the button.
     */
    public boolean getRawButton(int buttonNumber) {
        return pilot.getRawButton(buttonNumber);
    }

    /**
     * @return Returns triggers in controller.
     * <br>When the triggers are idle, 0 will be returned.
     * <br>When the right trigger is pressed, it will return a positive
     * value.
     * <br>When the left trigger is pressed, it will return a negative value.
     * <br>Therefore, both triggers pressed will return 0.
     */
    public double getTriggers() {
        double value;
        switch (controllerType) {
            case PS4:
                value = (pilot.getRawAxis(portsTriggersPS4[1]) - pilot.getRawAxis(portsTriggersPS4[0])) / 2;
                break;
            case XBOX:
                value = pilot.getRawAxis(portsTriggersXBOX[1]) - pilot.getRawAxis(portsTriggersXBOX[0]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getTriggers(). Returned 0. Use getRawAxis() instead.", false);
                break;
        }
        return ground(value, offset);
    }

    /**
     * By default, offset equals 0.1, meaning that if any of the axis is
     * equal to or less than 0.1 and greater than or equal to -0.1, it will
     * return 0.
     *
     * @param offset The offset that it will have on all axis.
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }

    /**
     * @return the amount of offset that it will correct.
     */
    public double getOffset() {
        return this.offset;
    }

    /**
     * This function will correct a given value according to the given offset.
     *
     * @param value  raw value.
     * @param offset positive double less than the value.
     * @return corrected value.
     */
    private double ground(double value, double offset) {
        return value >= -offset && value <= offset ? 0 : value;
    }

    /**
     * This method will add the buttons to buttons (private) according to the controllerType.
     */
    private void setButtons() {
        List<JoystickButton> bs = new ArrayList<JoystickButton>();
        switch (controllerType) {
            case XBOX:
                for (int port : portsButtonsXBOX) {
                    bs.add(new JoystickButton(pilot, port));
                }
                break;
            case PS4:
                for (int port : portsButtonsPS4) {
                    bs.add(new JoystickButton(pilot, port));
                }
                break;
            default:

                for (int i = 0; i < pilot.getButtonCount(); i++) {
                    bs.add(new JoystickButton(pilot, i + 1));
                }
                break;
        }
        buttons = (JoystickButton[]) bs.toArray();
    }


    /**
     * @param button the button to return
     * @return Returns JoystickButton Object
     */
    public JoystickButton getButton(ButtonType button) {
        int index = 0;
        switch (button) {
            case A:
                //index = 0; statement not necessary.
                break;
            case B:
                index = 1;
                break;
            case X:
                index = 2;
                break;
            case Y:
                index = 3;
                break;
            case LB:
                index = 4;
                break;
            case RB:
                index = 5;
                break;
            case BACK:
                index = 6;
                break;
            case START:
                index = 7;
                break;
            case LS:
                index = 8;
                break;
            case RS:
                index = 9;
                break;
            default:
                DriverStation.reportError("That's a problem.", false);
                break;
        }
        return buttons[index];

    }

    /**
     * Starts the given command whenever the button is newly pressed.
     *
     * @param button the button to be referred to.
     * @param command the command to start
     */
    public void whenPressed(ButtonType button, Command command) {
        JoystickButton m_button = getButton(button);
        m_button.whenPressed(command);
    }

    /**
     * Starts the command when the button is released.
     *
     * @param button the button to be referred to.
     * @param command the command to start
     */
    public void whenReleased(ButtonType button, Command command) {
        JoystickButton m_button = getButton(button);
        m_button.whenReleased(command);
    }

    /**
     * Constantly starts the given command while the button is held.
     *
     * {@link Command#start()} will be called repeatedly while the button is held, and will be
     * canceled when the button is released.
     *
     * @param button the button to be referred to.
     * @param command the command to start
     */
    public void whileHeld(ButtonType button, Command command) {
        JoystickButton m_button = getButton(button);
        m_button.whileHeld(command);
    }

    /**
     * Set the rumble output for the HID. The DS currently supports 2 rumble values, left rumble and
     * right rumble.
     *
     * @param type  Which rumble value to set
     * @param value The normalized value (0 to 1) to set the rumble to
     */
    public void setRumble(GenericHID.RumbleType type, double value) {
        pilot.setRumble(type, value);
    }

}
