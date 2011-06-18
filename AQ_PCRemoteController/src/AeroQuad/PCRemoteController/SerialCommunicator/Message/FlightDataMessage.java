package AeroQuad.PCRemoteController.SerialCommunicator.Message;

public class FlightDataMessage implements SerialMessage
{
    private int _motors[] = new int[4];
    private String _loopTime;

    public FlightDataMessage(final String data)
    {
        decode(data);
    }

    private void decode(final String data)
    {
//        System.out.println(data);
        final String[] datas = data.split(",");
        _loopTime = datas[0];
        _motors[0] = Integer.parseInt(datas[5]);
        _motors[1] = Integer.parseInt(datas[6]);
        _motors[2] = Integer.parseInt(datas[7]);
        _motors[3] = Integer.parseInt(datas[8]);
    }

    @Override
    public String getName()
    {
        return FLIGHT_DATA_MESSAGE_NAME;
    }

    public int getMotorThrottle(int idx)
    {
        return _motors[idx];
    }

    public String getLoopTime()
    {
        return _loopTime;
    }
}
