package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem instance;

    private Spark blinkin;
    private double color = LEDConstants.kLEDBlue;

    public LEDSubsystem() {
        blinkin = new Spark(Constants.LEDConstants.kPWMPort);
        blinkin.set(this.color);
    }

    public static synchronized LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    public void setColor(double color) {
        blinkin.set(color);
    }
}