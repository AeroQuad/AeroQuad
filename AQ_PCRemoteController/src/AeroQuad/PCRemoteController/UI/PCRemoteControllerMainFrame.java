package AeroQuad.PCRemoteController.UI;

import AeroQuad.PCRemoteController.GamePad.GamePadDetection.GamePadDetectionController;
import AeroQuad.PCRemoteController.GamePad.GamePadDetection.IGamePadDetectionController;
import AeroQuad.PCRemoteController.SerialCommunicator.ISerialCommunicator;
import AeroQuad.PCRemoteController.UI.ArtificialHorizon.ArtificialHorizonController;
import AeroQuad.PCRemoteController.UI.GamePad.GamePadPanel;
import AeroQuad.PCRemoteController.UI.GamePad.GamePadPanelController;
import AeroQuad.PCRemoteController.UI.GamePad.IGamePadPanelController;
import AeroQuad.PCRemoteController.UI.ShipConnectionPanel.ShipConnectionPanel;
import AeroQuad.PCRemoteController.UI.ShipConnectionPanel.ShipConnectionPanelController;
import AeroQuad.PCRemoteController.srcimport.ArtificialHorizon.PanelGUI;

import javax.swing.*;
import java.awt.*;

public class PCRemoteControllerMainFrame extends JFrame
{
    public PCRemoteControllerMainFrame(final ISerialCommunicator communicator)
    {
        initUi(communicator);
    }

    private void initUi(final ISerialCommunicator communicator)
    {
        final JPanel mainPanel = new JPanel(new BorderLayout());


        final ShipConnectionPanel connectionPanel = new ShipConnectionPanel(new ShipConnectionPanelController(communicator));
        final JPanel connectionPanelContainer = new JPanel(new GridLayout(2, 1));
        connectionPanelContainer.add(connectionPanel);

        final IGamePadDetectionController gamePadDetectionController = new GamePadDetectionController(communicator);
        final IGamePadPanelController gamePadPanelController = new GamePadPanelController(gamePadDetectionController,communicator);
        final GamePadPanel gamePadPanel = new GamePadPanel(gamePadPanelController);
        connectionPanelContainer.add(gamePadPanel, BorderLayout.CENTER);
        mainPanel.add(connectionPanelContainer);

        final PanelGUI haPanel = new PanelGUI("kalman");
        new ArtificialHorizonController(haPanel, communicator);
        mainPanel.add(haPanel, BorderLayout.WEST);

//        final MotorsPanel motorPanel = new MotorsPanel(new MotorsPanelController(communicator));
//        mainPanel.add(motorPanel, BorderLayout.SOUTH);
        getContentPane().add(mainPanel);
        pack();
        setVisible(true);
    }
}
