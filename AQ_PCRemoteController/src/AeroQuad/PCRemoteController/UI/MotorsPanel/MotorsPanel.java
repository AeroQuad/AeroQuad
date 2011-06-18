package AeroQuad.PCRemoteController.UI.MotorsPanel;


import javax.swing.*;
import java.awt.*;

public class MotorsPanel extends JPanel implements IMotorsPanel
{
    final static int MIN_MOTOR_THROTTLE = 1000;
    final static int MAX_MOTOR_THROTTLE = 2000;

    private final JSlider _motor1Slider = new JSlider(MIN_MOTOR_THROTTLE, MAX_MOTOR_THROTTLE);
    private final JSlider _motor2Slider = new JSlider(MIN_MOTOR_THROTTLE, MAX_MOTOR_THROTTLE);
    private final JSlider _motor3Slider = new JSlider(MIN_MOTOR_THROTTLE, MAX_MOTOR_THROTTLE);
    private final JSlider _motor4Slider = new JSlider(MIN_MOTOR_THROTTLE, MAX_MOTOR_THROTTLE);

    public MotorsPanel(final IMotorsPanelController controller)
    {
        controller.setPanel(this);

        setLayout(new GridLayout(1, 4));
        configureSlider(_motor1Slider);
        configureSlider(_motor2Slider);
        configureSlider(_motor3Slider);
        configureSlider(_motor4Slider);

    }

    private void configureSlider(final JSlider slider)
    {
        add(slider);
        slider.setValue(MIN_MOTOR_THROTTLE);
        slider.setOrientation(SwingConstants.VERTICAL);

        slider.setPaintTicks(true);
        slider.setMajorTickSpacing(100);
        slider.setMinorTickSpacing(5);
        slider.setPaintLabels(true);
        slider.setEnabled(false);
    }

    @Override
    public void setMotorValue(int i, int motorThrottle)
    {
        switch (i)
        {
            case 0:
                _motor1Slider.setValue(motorThrottle);
                break;
            case 1:
                _motor2Slider.setValue(motorThrottle);
                break;
            case 2:
                _motor3Slider.setValue(motorThrottle);
                break;
            case 3:
                _motor4Slider.setValue(motorThrottle);
                break;
            default:
        }
    }
}
