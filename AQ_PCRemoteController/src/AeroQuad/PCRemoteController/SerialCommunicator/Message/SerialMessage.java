package AeroQuad.PCRemoteController.SerialCommunicator.Message;

public interface SerialMessage
{
    public final String SENSORS_DATA_MESSAGE_NAME = "SENSORS_DATA_MESSAGE_NAME";
    public final String FLIGHT_DATA_MESSAGE_NAME = "FLIGHT_DATA_MESSAGE_NAME";

    String getName();
}
