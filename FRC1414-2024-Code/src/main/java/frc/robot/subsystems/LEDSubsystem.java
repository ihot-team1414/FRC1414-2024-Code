package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem instance;

    Spark blinken = new Spark(Constants.LEDConstants.kPWMPort);

    public LEDSubsystem() {
    }

    public static synchronized LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    public void setColor(double color) {
        blinken.set(color);
    }

    @Override
    public void periodic() {
    }
}