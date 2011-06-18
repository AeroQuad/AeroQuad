package AeroQuad.PCRemoteController.GamePad.GamePadDetection;

import AeroQuad.PCRemoteController.GamePad.GamePadController.SaitecEvoFlightStick;
import AeroQuad.PCRemoteController.GamePad.GamePadController.IGamePad;
import AeroQuad.PCRemoteController.GamePad.GamePadController.XBox360Pad;
import AeroQuad.PCRemoteController.SerialCommunicator.ISerialCommunicator;
import de.hardcode.jxinput.JXInputDevice;
import de.hardcode.jxinput.JXInputManager;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;

public class GamePadDetectionController implements IGamePadDetectionController
{
    private final PropertyChangeSupport _propertyChangeSupport = new PropertyChangeSupport(this);
    private final Thread _pollingThread = new Thread(new PollingTask());

    private IGamePad _gamePad;

    private boolean _detectDevice = true;

    private final ISerialCommunicator _serialCommunicator;

    public GamePadDetectionController(final ISerialCommunicator serialCommunicator)
    {
        _serialCommunicator = serialCommunicator;

       _pollingThread.start();
    }

    @Override
    public void addListener(final String propertyName, final PropertyChangeListener listener)
    {
        _propertyChangeSupport.addPropertyChangeListener(propertyName,listener);
    }


    private class PollingTask implements Runnable
    {
        @Override
        public void run()
        {
            while (_detectDevice)
            {
                try
                {
                    Thread.sleep(50);
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
                }
                pollGamePads();
            }
        }
    }



    private void pollGamePads()
    {
        if (_gamePad == null)
        {
            detectDevice();
        }
        else
        {
            _detectDevice = false;
        }
    }

    private void detectDevice()
    {
        JXInputManager.reset();
        for (int i = 0 ; i < JXInputManager.getNumberOfDevices(); i++)
        {
            final JXInputDevice device = JXInputManager.getJXInputDevice( i );
            if (device.getName().contains(XBOX_360_CONTROLLER_ID))
            {
                _gamePad = new XBox360Pad(device,_serialCommunicator);
                notifyDeviceDetected(_gamePad);
            }
            else if (device.getName().contains(SAITEK_CYBORG_EVO_ID))
            {
                _gamePad = new SaitecEvoFlightStick(device,_serialCommunicator);
                notifyDeviceDetected(_gamePad);
            }
            else
            {
                System.out.println("Unsuported device = " + device.getName());
            }
        }
    }

    private void notifyDeviceDetected(final IGamePad gamePad)
    {
        _propertyChangeSupport.firePropertyChange(GAME_PAD_UPDATED,null,gamePad);
    }

}
