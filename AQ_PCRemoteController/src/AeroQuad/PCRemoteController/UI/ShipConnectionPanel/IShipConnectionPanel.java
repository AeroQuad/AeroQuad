package AeroQuad.PCRemoteController.UI.ShipConnectionPanel;

public interface IShipConnectionPanel
{
    void setConnectionButtonEnabled(boolean enabled);

    void setDeconnectionButtonEnabled(boolean enabled);

    void setConnectionState(boolean connected);

    void setLoopTime(String loopTime);
}
