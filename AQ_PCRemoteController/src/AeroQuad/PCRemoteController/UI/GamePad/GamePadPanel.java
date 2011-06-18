package AeroQuad.PCRemoteController.UI.GamePad;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class GamePadPanel extends JPanel implements IGamePadPanel
{
    final IGamePadPanelController _controller;

    final JLabel _connectionState = new JLabel();
    final JLabel _gamePadName = new JLabel();
    final JButton _calibrateButton = new JButton("CALIBRATE");

    public GamePadPanel(final IGamePadPanelController controller)
    {
        _controller = controller;
        _controller.setPanel(this);
        setLayout(new GridLayout(6, 1));

        add(new JLabel("GAME CONTROLLER"));
        initConnectionStateLabel();
        add(_gamePadName);
        add(_calibrateButton);
        _calibrateButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                _controller.requestCalibrate();
            }
        });
    }

    private void initConnectionStateLabel()
    {
        _connectionState.setText(UNDETECTED);
        _connectionState.setOpaque(true);
        _connectionState.setBackground(Color.red);
        add(_connectionState);
    }


    @Override
    public void setGamePadConnected(final boolean connected)
    {
        if (connected)
        {
            _connectionState.setText(CONNECTED);
            _connectionState.setBackground(Color.GREEN);
        }
        else
        {
            _connectionState.setText(UNDETECTED);
            _connectionState.setBackground(Color.RED);
        }
    }

    @Override
    public void setGamePadName(final String name)
    {
        _gamePadName.setText(name);
    }


}
