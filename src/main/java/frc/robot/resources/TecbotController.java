package frc.robot.resources;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

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

    private Joystick pilot;
    TypeOfController controllerType;
    JoystickButton[] buttons;
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
                DriverStation.reportWarning("Could not get axis value from getLeftAxisX(). Returned 0. Returned 0. Use getAxisValue() instead.", false);
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
                DriverStation.reportWarning("Could not get axis value from getLeftAxisY(). Returned 0. Returned 0. Use getAxisValue() instead.", false);
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
                DriverStation.reportWarning("Could not get axis value from getRightAxisX(). Returned 0. Returned 0. Use getAxisValue() instead.", false);
                break;
        }
        return ground(value, offset);
    }

    /**
     * This function will return the value of the Right Axis <i>Y</i>.
     * <br>Ranges from -1 to 1.
     *
     * @return axis value
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
                DriverStation.reportWarning("Could not get axis value from getRightAxisY(). Returned 0. Use getAxisValue() instead.", false);
                break;
        }
        return ground(value, getOffset());
    }

    /**
     * Returns value of given axis.
     *
     * @param axis axis port in the controller.
     * @return value of given axis.
     */
    public double getAxisValue(int axis, boolean ground) {
        return ground ? ground(pilot.getRawAxis(axis), offset) : pilot.getRawAxis(axis);
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
                DriverStation.reportWarning("Could not get axis value from getTriggers(). Returned 0. Use getAxisValue() instead.", false);
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

    private void setButtons() {
        switch (controllerType) {
            case XBOX:
                buttons = new JoystickButton[]{
                        new JoystickButton(pilot, portsButtonsXBOX[0]),
                        new JoystickButton(pilot, portsButtonsXBOX[1]),
                        new JoystickButton(pilot, portsButtonsXBOX[2]),
                        new JoystickButton(pilot, portsButtonsXBOX[3]),
                        new JoystickButton(pilot, portsButtonsXBOX[4]),
                        new JoystickButton(pilot, portsButtonsXBOX[5]),
                        new JoystickButton(pilot, portsButtonsXBOX[6]),
                        new JoystickButton(pilot, portsButtonsXBOX[7]),
                        new JoystickButton(pilot, portsButtonsXBOX[8]),
                        new JoystickButton(pilot, portsButtonsXBOX[9])
                };
                break;
            case PS4:
                buttons = new JoystickButton[]{
                        new JoystickButton(pilot, portsButtonsPS4[0]),
                        new JoystickButton(pilot, portsButtonsPS4[1]),
                        new JoystickButton(pilot, portsButtonsPS4[2]),
                        new JoystickButton(pilot, portsButtonsPS4[3]),
                        new JoystickButton(pilot, portsButtonsPS4[4]),
                        new JoystickButton(pilot, portsButtonsPS4[5]),
                        new JoystickButton(pilot, portsButtonsPS4[6]),
                        new JoystickButton(pilot, portsButtonsPS4[7]),
                        new JoystickButton(pilot, portsButtonsPS4[8]),
                        new JoystickButton(pilot, portsButtonsPS4[9])
                };
                break;
            default:
                List<JoystickButton> bs = new List<JoystickButton>() {
                    @Override
                    public int size() {
                        return 0;
                    }

                    @Override
                    public boolean isEmpty() {
                        return false;
                    }

                    @Override
                    public boolean contains(Object o) {
                        return false;
                    }

                    @Override
                    public Iterator<JoystickButton> iterator() {
                        return null;
                    }

                    @Override
                    public Object[] toArray() {
                        return new Object[0];
                    }

                    @Override
                    public <T> T[] toArray(T[] a) {
                        return null;
                    }

                    @Override
                    public boolean add(JoystickButton joystickButton) {
                        return false;
                    }

                    @Override
                    public boolean remove(Object o) {
                        return false;
                    }

                    @Override
                    public boolean containsAll(Collection<?> c) {
                        return false;
                    }

                    @Override
                    public boolean addAll(Collection<? extends JoystickButton> c) {
                        return false;
                    }

                    @Override
                    public boolean addAll(int index, Collection<? extends JoystickButton> c) {
                        return false;
                    }

                    @Override
                    public boolean removeAll(Collection<?> c) {
                        return false;
                    }

                    @Override
                    public boolean retainAll(Collection<?> c) {
                        return false;
                    }

                    @Override
                    public void clear() {

                    }

                    @Override
                    public JoystickButton get(int index) {
                        return null;
                    }

                    @Override
                    public JoystickButton set(int index, JoystickButton element) {
                        return null;
                    }

                    @Override
                    public void add(int index, JoystickButton element) {

                    }

                    @Override
                    public JoystickButton remove(int index) {
                        return null;
                    }

                    @Override
                    public int indexOf(Object o) {
                        return 0;
                    }

                    @Override
                    public int lastIndexOf(Object o) {
                        return 0;
                    }

                    @Override
                    public ListIterator<JoystickButton> listIterator() {
                        return null;
                    }

                    @Override
                    public ListIterator<JoystickButton> listIterator(int index) {
                        return null;
                    }

                    @Override
                    public List<JoystickButton> subList(int fromIndex, int toIndex) {
                        return null;
                    }
                };
                for (int i = 0; i < pilot.getButtonCount(); i++) {
                    bs.add(new JoystickButton(pilot, i + 1));
                }
                buttons = (JoystickButton[]) bs.toArray();
                break;
        }
    }


    /**
     *
     * @param button the button to return
     * @return Returns JoystickButton Object
     */
    public JoystickButton getButton(ButtonType button){
        int index = 0;
        switch(button){
            case A:
                index = 0;
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


}
