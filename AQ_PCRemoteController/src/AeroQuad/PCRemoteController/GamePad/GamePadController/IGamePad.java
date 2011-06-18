package AeroQuad.PCRemoteController.GamePad.GamePadController;

public interface IGamePad
{
    public static int ROLL = 0;
    public static int PITCH = 1;
    public static int YAW = 2;
    public static int THROTTLE = 3;
    public static int MODE = 4;
    public static int AUX = 5;

    String getName();
}
