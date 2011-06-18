package AeroQuad.PCRemoteController.SerialCommunicator.Message;

public class MessageFactory implements IMessageFactory
{

    @Override
    public SerialMessage createMessage(final int nbComma, final String data)
    {
        switch (nbComma)
        {
            case SENSORS_MESSAGE:
                return new SensorsDataMessage(data);
            case FLIGHT_DATA_MESSAGE:
                return new FlightDataMessage(data);
        }
        return null;  //To change body of implemented methods use File | Settings | File Templates.
    }
}
