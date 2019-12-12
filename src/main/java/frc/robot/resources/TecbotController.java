package frc.robot.resources;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/**
 * A Tecbot Controller is a Joystick controller in which you can get different values from your controller,
 * whether it is a Ps4, Ps3, Xbox ONE, Xbox 360 or any other controller.
 */
public class TecbotController extends Joystick {


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
    private Joystick pilot;
    TypeOfController controllerType;
    private double offset = 0.1;

    private enum TypeOfController {
        PS4,
        XBOX
    }

    /**
     *
     * @param port The port that the controller has in the Driver Station.
     */
    public TecbotController(int port) {
        super(port);
        pilot = new Joystick(port);
        controllerType = null;
        if (pilot.getName().toLowerCase().contains("wireless controller")) controllerType = TypeOfController.PS4;
        if (pilot.getName().toLowerCase().contains("xbox")) controllerType = TypeOfController.XBOX;

        if (pilot == null) DriverStation.reportWarning("Joystick not found (Tecbot Controller)", false);
    }

    /**
     * This function will return the value of the Left Axis <i>Y</i>.
     * <br>Ranges from -1 to 1.
     *
     * @return axis value
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
                DriverStation.reportWarning("Could not get axis value from getLeftAxisX(). Returned 0.", false);
                break;
        }
        return ground(value, getOffset());
    }

    /**
     * This function will return the value of the Left Axis <i>Y</i>.
     * <br>Ranges from -1 to 1.
     *
     * @return axis value
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
                DriverStation.reportWarning("Could not get axis value from getLeftAxisY(). Returned 0.", false);
                break;
        }
        return ground(value, getOffset());
    }


    /**
     * This function will return the value of the Right Axis <i>X</i>.
     * <br>Ranges from -1 to 1.
     *
     * @return axis value
     */
    public double getRightAxisX() {
        double value = 0;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[2]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[2]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getRightAxisX(). Returned 0.", false);
                break;
        }
        return ground(value, getOffset());
    }

    /**
     * This function will return the value of the Right Axis <i>Y</i>.
     * <br>Ranges from -1 to 1.
     *
     * @return axis value
     */
    public double getRightAxisY() {
        double value = 0;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[3]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[3]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getRightAxisY(). Returned 0.", false);
                break;
        }
        return ground(value, getOffset());
    }

    /**
     * Returns value of given axis.
     * @param axis axis port in the controller.
     * @return value of given axis.
     */
    public double getAxisValue(int axis, boolean ground) {
        return ground ? ground(pilot.getRawAxis(axis), offset) : pilot.getRawAxis(axis);
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
        return value >= -0.1 && value <= 0.1 ? 0 : value;
    }


}
