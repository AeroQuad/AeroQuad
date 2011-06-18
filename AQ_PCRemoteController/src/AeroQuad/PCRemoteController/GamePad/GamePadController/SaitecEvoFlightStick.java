package AeroQuad.PCRemoteController.GamePad.GamePadController;

import AeroQuad.PCRemoteController.SerialCommunicator.ISerialCommunicator;
import AeroQuad.PCRemoteController.Utils.MathUtils;
import de.hardcode.jxinput.Axis;
import de.hardcode.jxinput.Directional;
import de.hardcode.jxinput.JXInputDevice;

public class SaitecEvoFlightStick extends GamePad
{
    private final String X_AXIS = "X Axis";
    private final String Y_AXIS = "Y Axis";
    private final String Z_AXIS = "Z Axis";
    private final String Z_ROTATION = "Z Rotation";




    public SaitecEvoFlightStick(final JXInputDevice device,final ISerialCommunicator serialCommunicator)
    {
        super(device,serialCommunicator);

        // value hardcoded for now stable/not altitude hold
//        _transmitterValue[MODE] = 2000;
//        _transmitterValue[AUX] = 2000;
    }

    @Override
    public String getName()
    {
        return "Flight Stick";
    }



    @Override
    protected void updateDirectionnalValue(final Directional directional)
    {
        // do nothing here
    }

    @Override
    protected void updateAxisValue(final Axis axis)
    {
        if (axis.getName().equals(X_AXIS))
        {
            _transmitterValue[ROLL] = (int)MathUtils.map(axis.getValue(),-1,1,1000,2000);
        }
        else if (axis.getName().equals(Y_AXIS))
        {
            _transmitterValue[PITCH] = (int)MathUtils.map(axis.getValue(),1,-1,1000,2000);
        }
        if (axis.getName().equals(Z_AXIS))
        {
            _transmitterValue[THROTTLE] = (int)MathUtils.map(axis.getValue(),1,-1,1000,2000);
        }
        else if (axis.getName().equals(Z_ROTATION))
        {
            _transmitterValue[YAW] = (int)MathUtils.map(axis.getValue(),-1,1,1000,2000);
        }
    }
}
