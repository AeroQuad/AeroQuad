package AeroQuad.PCRemoteController.GamePad.GamePadController;

import AeroQuad.PCRemoteController.SerialCommunicator.ISerialCommunicator;
import de.hardcode.jxinput.Axis;
import de.hardcode.jxinput.Directional;
import de.hardcode.jxinput.JXInputDevice;

public class XBox360Pad extends GamePad
{
    public XBox360Pad(final JXInputDevice device, final ISerialCommunicator serialCommunicator)
    {
        super(device, serialCommunicator);
    }

    @Override
    public String getName()
    {
        return "Game Pad";
    }

    @Override
    protected void updateDirectionnalValue(Directional directional)
    {
        //To change body of implemented methods use File | Settings | File Templates.
    }

    @Override
    protected void updateAxisValue(Axis axis)
    {
        //To change body of implemented methods use File | Settings | File Templates.
    }
}
