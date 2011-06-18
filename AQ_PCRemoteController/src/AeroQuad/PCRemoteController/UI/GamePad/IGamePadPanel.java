package AeroQuad.PCRemoteController.UI.GamePad;

public interface IGamePadPanel
{
    public static String CONNECTED = "CONNECTED";
    public static String UNDETECTED = "UNDETECTED";

    void setGamePadConnected(boolean connected);

    void setGamePadName(String name);
}
