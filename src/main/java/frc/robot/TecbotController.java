package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/**
 * A Tecbot Controller is a Joystick controller in which you can get different values from your controller,
 * whether it is a Ps4, Ps3, Xbox ONE, Xbox 360 or any other controller.
 */
public class TecbotController extends Joystick {


    private int[] portsJoysticksPS4 = {0,1,2,5};
    private int[] portsJoystickXbox = {0,1,4,5};
    private int leftJoystickX = 0, leftJoystickY = 0, rightJoystickX = 0, rightJoystickY = 0;
    private Joystick pilot;
    TypeOfController controllerType;

    private enum TypeOfController{
        PS4,
        XBOX,
        OTHER
    }

    public enum AxisJoystick{
        LEFT_X,
        LEFT_Y,
        RIGHT_X,
        RIGHT_Y
    }

    public TecbotController(int port) {
        super(port);
        pilot = new Joystick(port);
        controllerType = TypeOfController.OTHER;
        if(pilot.getName().toLowerCase().contains("wireless controller")) controllerType = TypeOfController.PS4;
        if(pilot.getName().toLowerCase().contains("xbox")) controllerType = TypeOfController.XBOX;

        if(pilot==null) DriverStation.reportWarning("Joystick not found (Tecbot Controller)",false);
    }

    public double getLeftX(){
        double value = 0;
        switch (controllerType){
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[0]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXbox[0]);
                break;
            case OTHER:
                value = pilot.getRawAxis(0);
                break;
            default:
                value = 0;
                break;

        }
        return value;
    }



}
