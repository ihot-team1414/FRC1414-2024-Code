package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.LEDConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem instance;

    private Spark blinkin;
    private double color = LEDConstants.kLEDBlue;

    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

    public LEDSubsystem() {
        blinkin = new Spark(LEDConstants.kPWMPort);
        blinkin.set(this.color);
    }

    public static synchronized LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }


    public boolean isReadyForNote(){
        return shooter
            .isWithinVelocityTolerance(400) ? true : false;
    }

    public void setColor(double color) {
        blinkin.set(color);
    }

    public Command changeColor(double color) {
        return this.run(() -> setColor(color));
    }

    @Override
    public void periodic() {
        if (!intake.isLoaded()){blinkin.set(LEDConstants.kLEDBlue);}
    }
}