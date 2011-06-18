package AeroQuad.PCRemoteController.SerialCommunicator;

import AeroQuad.PCRemoteController.SerialCommunicator.Message.IMessageFactory;
import AeroQuad.PCRemoteController.SerialCommunicator.Message.MessageFactory;
import AeroQuad.PCRemoteController.SerialCommunicator.Message.SerialMessage;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

import javax.swing.*;
import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

public class SerialCommunicator implements ISerialCommunicator
{
    final String[] commPorstIds = {"COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9"};

    final IMessageFactory _messageFactory = new MessageFactory();

    final PropertyChangeSupport _properPropertyChangeSupport = new PropertyChangeSupport(this);

    private InputStream _inputStream;
    private OutputStream _outputStream;
    private SerialPort _serialPort;
    private CommPortIdentifier _comPortId;
    private boolean _connected = false;

    private final Object _mutex = new Object();
    private String _transmitterValue;
    private final Thread _connectionThread = new ConnectionThread();
    private long _timeLastReceived = 0;
    private String _specialRequest;




    @Override
    public List<String> getComPortAvailable()
    {
        final List<String> ret = new ArrayList<String>();
        for (final String comPortId : commPorstIds)
        {
            try
            {
                CommPortIdentifier.getPortIdentifier(comPortId);
                ret.add(comPortId);
            } catch (Exception e)
            {
                System.out.println(comPortId + " unavailable");
            }
        }

        return ret;
    }

    @Override
    public void connect(final String commPort, final String commSpeed)
    {
        try
        {
            synchronized (_mutex)
            {
                _comPortId = CommPortIdentifier.getPortIdentifier(commPort);
                _serialPort = (SerialPort) _comPortId.open("ComPort", 2000);
                _inputStream = _serialPort.getInputStream();
                _outputStream = _serialPort.getOutputStream();
                _serialPort.addEventListener(new MySerialCommPortEventListener());
                _serialPort.notifyOnDataAvailable(true);
                final int speed = Integer.parseInt(commSpeed);
                _serialPort.setSerialPortParams(speed, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
                _connected = true;
                Thread.sleep(3000);
                notifyConnectionStatus(true);

                _connectionThread.start();
            }
        }
        catch (Exception e)
        {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        }
    }

    @Override
    public void addListener(final String propertyName, final PropertyChangeListener listener)
    {
        _properPropertyChangeSupport.addPropertyChangeListener(propertyName,listener);
    }

    @Override
    public void sendSimulatedTransmitterValue(final String transmitterValue)
    {
        _transmitterValue = transmitterValue;
    }

    @Override
    public void requestSpecialCommand(final String command)
    {
        _specialRequest = command;
    }

    private void notifyConnectionStatus(final boolean connected)
    {
        _properPropertyChangeSupport.firePropertyChange(CONNECTION_STATE_CHANGE, null, connected);
    }

    private class MySerialCommPortEventListener implements SerialPortEventListener
    {
        public void serialEvent(final SerialPortEvent serialPortEvent)
        {
            handleSerialCommEvent(serialPortEvent);
        }
    }

    private void handleSerialCommEvent(final SerialPortEvent serialPortEvent)
    {
        switch (serialPortEvent.getEventType())
        {
            case SerialPortEvent.BI:
            case SerialPortEvent.OE:
            case SerialPortEvent.FE:
            case SerialPortEvent.PE:
            case SerialPortEvent.CD:
            case SerialPortEvent.CTS:
            case SerialPortEvent.DSR:
            case SerialPortEvent.RI:
            case SerialPortEvent.OUTPUT_BUFFER_EMPTY:
                break;
            case SerialPortEvent.DATA_AVAILABLE:
                final StringBuffer readBuffer = new StringBuffer();
                int c;
                try
                {
                    while ((c = _inputStream.read()) != 10)
                    {
                        if (c != 13)
                        {
                            readBuffer.append((char) c);
                        }
                    }
                    _inputStream.close();

                    notifyDataReceived(readBuffer.toString());
                } catch (IOException e)
                {
                    e.printStackTrace();
                }
                break;
        }
    }

    private void notifyDataReceived(final String data)
    {
        final int nbComma = countComma(data);
        final SerialMessage message = _messageFactory.createMessage(nbComma,data);
        if (message != null)
        {
            SwingUtilities.invokeLater(new Runnable()
            {
                public void run()
                {
                    _properPropertyChangeSupport.firePropertyChange(message.getName(), null, message);
                }
            });
        }
        else
        {
            System.out.println("Unsuported Message = " + nbComma + " " + data);
        }
        _timeLastReceived = System.currentTimeMillis();
    }




    private int countComma(final String data)
    {
        return data.replaceAll("[^,]", "").length();
    }

    private void sendMessage(final String message)
    {
        try
        {
//            _outputStream.flush();
            _outputStream.write(message.getBytes());
            _outputStream.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        }
    }

    private class ConnectionThread extends Thread
    {
        public void run()
        {
            while(_connected)
            {
                try
                {
                    Thread.sleep(150);
                    if (_specialRequest != null)
                    {
                        for (final char c : _specialRequest.toCharArray())
                        {
                            final StringBuffer buf = new StringBuffer();
                            buf.append(c);
                            sendMessage(buf.toString());
                            Thread.sleep(2000);
                        }
                        _specialRequest = null;

                    }
                    else if (_transmitterValue != null)
                    {
                        sendMessage(_transmitterValue);
                        _transmitterValue = null;
                    }

                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
