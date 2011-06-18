package AeroQuad.PCRemoteController.UI.ShipConnectionPanel;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.List;

public class ShipConnectionPanel extends JPanel implements IShipConnectionPanel
{
    private final String[] _comPortSpeedStrings = {"4800","9600","14400","19200","28800","38400","57600","115200"};

    private final JComboBox _comPortComboBox = new JComboBox();
    private final JComboBox _comPortSpeedComboBox = new JComboBox();
    final JButton _connectionButton = new JButton("CONNECT");
    final JButton _disconnectButton = new JButton("DISCONNECT");
    final JLabel _connectionStateLabel = new JLabel("DISCONNECTED");
    final JLabel _loopTimeLabel = new JLabel();

    private IShipConnectionPanelController _controller;

    public ShipConnectionPanel(final IShipConnectionPanelController controller)
    {
        _controller = controller;
        _controller.setPanel(this);
        setLayout(new GridLayout(7, 1));
        initComboBox();

        final JLabel header = new JLabel("SHIP");
        header.setHorizontalAlignment(SwingConstants.CENTER);
        add(header);
        add(_comPortComboBox);
        add(_comPortSpeedComboBox);
        add(_connectionButton);
        add(_disconnectButton);
        _connectionStateLabel.setOpaque(true);
        _connectionStateLabel.setBackground(Color.red);
        add(_connectionStateLabel);
        add(_loopTimeLabel);

        initButtonAction();
    }

    private void initComboBox()
    {
        fillCommPortComboBox();
        _comPortComboBox.setSelectedIndex(_comPortComboBox.getItemCount()-1);

        for (final String comPortSpeed:_comPortSpeedStrings)
        {
            _comPortSpeedComboBox.addItem(comPortSpeed);
        }
        _comPortSpeedComboBox.setSelectedIndex(_comPortSpeedComboBox.getItemCount() - 1);
    }

    private void fillCommPortComboBox()
    {
        final List<String> availableComPort = _controller.getComPortAvailable();
        for (final String comPort:availableComPort)
        {
            _comPortComboBox.addItem(comPort);
        }
    }

    private void initButtonAction()
    {
        _connectionButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
                final String commPort = (String) _comPortComboBox.getItemAt(_comPortComboBox.getSelectedIndex());
                if (commPort == null)
                {
                    return;
                }
                final String speed = (String) _comPortSpeedComboBox.getItemAt(_comPortSpeedComboBox.getSelectedIndex());
                _controller.connect(commPort,speed);
            }
        });
        _disconnectButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {

            }
        });
    }


    @Override
    public void setConnectionButtonEnabled(final boolean enabled)
    {
        _connectionButton.setEnabled(enabled);
    }

    @Override
    public void setDeconnectionButtonEnabled(final boolean enabled)
    {
        _disconnectButton.setEnabled(enabled);
    }

    @Override
    public void setConnectionState(final boolean connected)
    {
        if (connected)
        {
            _connectionStateLabel.setText("CONNECTED");
            _connectionStateLabel.setBackground(Color.GREEN);
        }
        else
        {
            _connectionStateLabel.setText("DISCONNECTED");
            _connectionStateLabel.setBackground(Color.RED);
        }
    }

    @Override
    public void setLoopTime(String loopTime)
    {
        _loopTimeLabel.setText(loopTime);
    }
}
