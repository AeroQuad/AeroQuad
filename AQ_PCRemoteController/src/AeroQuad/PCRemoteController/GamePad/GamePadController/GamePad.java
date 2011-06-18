package AeroQuad.PCRemoteController.GamePad.GamePadController;

import AeroQuad.PCRemoteController.SerialCommunicator.ISerialCommunicator;
import de.hardcode.jxinput.Axis;
import de.hardcode.jxinput.Directional;
import de.hardcode.jxinput.JXInputDevice;
import de.hardcode.jxinput.JXInputManager;

import javax.swing.*;

public abstract class GamePad implements IGamePad
{
    protected final int[] _transmitterValue = new int[4];

    private final JXInputDevice _device;

    private final ISerialCommunicator _serialCommunicator;

    public GamePad(final JXInputDevice device, ISerialCommunicator serialCommunicator)
    {
        _device = device;
        _serialCommunicator = serialCommunicator;
        _deviceUpdateThread.start();

        _transmitterValue[ROLL] = 1500;
        _transmitterValue[PITCH] = 1500;
        _transmitterValue[YAW] = 1500;
        _transmitterValue[THROTTLE] = 1000;
    }


    final Thread _deviceUpdateThread = new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            while (true)
            {
                try
                {
                    Thread.sleep(100);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
                }
                SwingUtilities.invokeLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        JXInputManager.updateFeatures();
                        handleUpdate();
                    }
                });
            }
        }
    });

    private void handleUpdate()
    {
        for (int i = 0; i < _device.getMaxNumberOfAxes(); i++)
        {
            if (_device.getAxis(i) != null)
            {
                updateAxisValue(_device.getAxis(i));
            }
        }
        for (int i = 0; i < _device.getNumberOfDirectionals(); i++)
        {
            if (_device.getDirectional(i) != null)
            {
                updateDirectionnalValue(_device.getDirectional(i));
            }
        }
        updateValueFinished();
    }

    protected abstract void updateDirectionnalValue(Directional directional);

    protected abstract void updateAxisValue(Axis axis);

    private void updateValueFinished()
    {
        final StringBuffer buffer = new StringBuffer();
        buffer.append(">");
        for (byte channel = ROLL; channel < MODE; channel++)  // send just ROLL,PITCH,YAW,THROTTLE
        {
            buffer.append(Integer.toString(_transmitterValue[channel]));
            if (channel < THROTTLE)
            {
                buffer.append(';');
            }
        }
        _serialCommunicator.sendSimulatedTransmitterValue(buffer.toString());
//        System.out.println(buffer.toString());
    }
}
