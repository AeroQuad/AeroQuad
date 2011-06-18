package AeroQuad.PCRemoteController.SerialCommunicator;


import java.beans.PropertyChangeListener;
import java.util.*;

public interface ISerialCommunicator
{
    public static String CONNECTION_STATE_CHANGE = "CONNECTION_STATE_CHANGE";

    List<String> getComPortAvailable();

    void connect(String commPort, String speed);

    void addListener(String propertyName, PropertyChangeListener listener);

    void sendSimulatedTransmitterValue(String transmitterValue);

    void requestSpecialCommand(String command);
}
