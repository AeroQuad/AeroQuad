package AeroQuad.PCRemoteController.UI.ShipConnectionPanel;

import AeroQuad.PCRemoteController.SerialCommunicator.ISerialCommunicator;
import AeroQuad.PCRemoteController.SerialCommunicator.Message.FlightDataMessage;
import AeroQuad.PCRemoteController.SerialCommunicator.Message.SerialMessage;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.util.List;

public class ShipConnectionPanelController implements IShipConnectionPanelController
{
    private final ISerialCommunicator _communicator;
    private IShipConnectionPanel _panel;

    public ShipConnectionPanelController(final ISerialCommunicator communicator)
    {
        _communicator = communicator;

        _communicator.addListener(ISerialCommunicator.CONNECTION_STATE_CHANGE,new PropertyChangeListener()
        {
            @Override
            public void propertyChange(final PropertyChangeEvent event)
            {
                final boolean connected = ((Boolean)event.getNewValue()).booleanValue();
                updateConnectionState(connected);
            }
        });

        _communicator.addListener(SerialMessage.FLIGHT_DATA_MESSAGE_NAME,new PropertyChangeListener()
        {
            @Override
            public void propertyChange(final PropertyChangeEvent event)
            {
                final FlightDataMessage message = (FlightDataMessage)event.getNewValue();
                _panel.setLoopTime(message.getLoopTime());
            }
        });
    }

    private void updateConnectionState(final boolean connected)
    {
        _panel.setConnectionButtonEnabled(!connected);
        _panel.setDeconnectionButtonEnabled(connected);
        _panel.setConnectionState(connected);
    }

    public List<String> getComPortAvailable()
    {
        return _communicator.getComPortAvailable();
    }

    @Override
    public void connect(final String commPort, final String speed)
    {
        _communicator.connect(commPort,speed);
    }

    public void setPanel(IShipConnectionPanel panel)
    {
        _panel = panel;
        _panel.setConnectionButtonEnabled(true);
        _panel.setDeconnectionButtonEnabled(false);
    }
}
