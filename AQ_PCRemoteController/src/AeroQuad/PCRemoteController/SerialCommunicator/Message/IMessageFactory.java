package AeroQuad.PCRemoteController.SerialCommunicator.Message;

public interface IMessageFactory
{
    public static final int SENSORS_MESSAGE = 12;
    public static final int FLIGHT_DATA_MESSAGE = 19;


    SerialMessage createMessage(int nbComma, String data);
}
