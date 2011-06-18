package AeroQuad.PCRemoteController.GamePad.GamePadDetection;

import java.beans.PropertyChangeListener;

public interface IGamePadDetectionController
{
    public final String XBOX_360_CONTROLLER_ID = "XBOX 360 For Windows";
    public final String SAITEK_CYBORG_EVO_ID =  "Saitek Cyborg Evo";

    public static String GAME_PAD_UPDATED = "GAME_PAD_UPDATED";

    void addListener(String propertyName, PropertyChangeListener listener);
}
