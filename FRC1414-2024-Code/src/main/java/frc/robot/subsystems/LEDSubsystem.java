package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem instance;
    private double color;

    static Spark blinken = new Spark(Constants.LEDConstants.kPWMPort);

    public LEDSubsystem() {
        this.color = Constants.LEDConstants.kLEDBlue;
    }

    public void setColor(double colorChange) {
        this.color = colorChange;
    }

    public static synchronized LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;

    }

    @Override
    public void periodic() {
        blinken.set(color);
    }

}