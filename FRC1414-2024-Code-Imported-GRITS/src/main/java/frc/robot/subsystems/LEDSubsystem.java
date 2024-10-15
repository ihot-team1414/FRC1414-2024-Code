package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public Command set(DoubleSupplier color) {
        return new InstantCommand(() -> setColor(color.getAsDouble()), LEDSubsystem.getInstance());
    }
}