package AeroQuad.PCRemoteController;

import AeroQuad.PCRemoteController.SerialCommunicator.ISerialCommunicator;
import AeroQuad.PCRemoteController.SerialCommunicator.SerialCommunicator;
import AeroQuad.PCRemoteController.UI.PCRemoteControllerMainFrame;

import javax.swing.*;

public class Starter
{
    public Starter()
    {
        init();
    }

    private void init()
    {
        final ISerialCommunicator communicator = new SerialCommunicator();
        final PCRemoteControllerMainFrame mainFrame = new PCRemoteControllerMainFrame(communicator);
        mainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }


    public static void main(String[] args)
    {
        SwingUtilities.invokeLater(new Runnable()
        {
            public void run()
            {
                new Starter();
            }
        });
    }
}
