package AeroQuad.PCRemoteController.UI.GamePad;

import AeroQuad.PCRemoteController.GamePad.GamePadController.IGamePad;
import AeroQuad.PCRemoteController.GamePad.GamePadDetection.IGamePadDetectionController;
import AeroQuad.PCRemoteController.SerialCommunicator.ISerialCommunicator;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;

public class GamePadPanelController implements IGamePadPanelController
{
    private IGamePadPanel _panel;

    private final ISerialCommunicator _serialCommunicator;

    public GamePadPanelController(final IGamePadDetectionController gamePadDetectionController, final ISerialCommunicator serialCommunicator)
    {
        _serialCommunicator = serialCommunicator;
        gamePadDetectionController.addListener(IGamePadDetectionController.GAME_PAD_UPDATED, new PropertyChangeListener()
        {
            @Override
            public void propertyChange(final PropertyChangeEvent event)
            {
                final IGamePad gamePad = (IGamePad) event.getNewValue();
                updateGamePadData(gamePad);
            }
        });
    }

    private void updateGamePadData(final IGamePad gamePad)
    {
        _panel.setGamePadConnected(true);
        _panel.setGamePadName(gamePad.getName());
    }

    @Override
    public void setPanel(IGamePadPanel panel)
    {
        _panel = panel;
    }

    @Override
    public void requestCalibrate()
    {
        _serialCommunicator.requestSpecialCommand("bc");
    }
}
