package AeroQuad.PCRemoteController.UI.ShipConnectionPanel;

import java.util.List;

public interface IShipConnectionPanelController
{
    void setPanel(IShipConnectionPanel connectionPanel);

    List<String> getComPortAvailable();

    void connect(String commPort, String speed);
}
