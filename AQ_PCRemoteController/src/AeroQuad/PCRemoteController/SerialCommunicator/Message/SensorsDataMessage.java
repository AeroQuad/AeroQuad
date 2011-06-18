package AeroQuad.PCRemoteController.SerialCommunicator.Message;

public class SensorsDataMessage implements SerialMessage
{
    private float _roll;
    private float _pitch;

    public SensorsDataMessage(final String data)
    {
        decode(data);
    }

    @Override
    public String getName()
    {
        return SENSORS_DATA_MESSAGE_NAME;
    }


    private void decode(final String data)
    {
        final String[] datas = data.split(",");
        _roll = Float.parseFloat(datas[8]);
        _pitch = Float.parseFloat(datas[9]);
    }

    public float getRoll()
    {
        return _roll;
    }

    public float getPitch()
    {
        return _pitch;
    }


}
