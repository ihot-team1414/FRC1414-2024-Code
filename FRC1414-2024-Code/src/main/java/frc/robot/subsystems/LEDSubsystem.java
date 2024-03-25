package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.utils.RobotState;
import frc.utils.RobotState.RobotConfiguration;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem instance;
    private double color;

    static Spark blinkin = new Spark(Constants.LEDConstants.kPWMPort);

    public LEDSubsystem() {
        this.color = LEDConstants.kLEDBlue;
    }

    public void setColor(double newColor) {
        this.color = newColor;
    }

    public static synchronized LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;

    }

    @Override
    public void periodic() {

        if (IntakeSubsystem.getInstance().isLoaded()) {
            color = LEDConstants.kLEDOrange;
        } else {
            color = LEDConstants.kLEDBlue;
        }

        if (RobotState.getInstance().getRobotConfiguration() == RobotConfiguration.AMP) {
            color = LEDConstants.kLEDGold;
        }

        if (RobotState.getInstance().getRobotConfiguration() == RobotConfiguration.EJECTING) {
            color = LEDConstants.kLEDStrobeRed;
        }

        if (RobotState.getInstance().getRobotConfiguration() == RobotConfiguration.LIMELIGHT_SEARCHING) {
            color = LEDConstants.kLEDLightChaseRed;
        }

        if (RobotState.getInstance().getRobotConfiguration() == RobotConfiguration.AIMING_SUCCESS) {
            color = LEDConstants.kLEDGreenFlashing;
        }

        if (RobotState.getInstance().getRobotConfiguration() == RobotConfiguration.SHOOTING) {
            color = LEDConstants.kLEDGreen;
        }

        if (RobotState.getInstance().getRobotConfiguration() == RobotConfiguration.DISABLED) {
            color = LEDConstants.kLEDRainbow;
        }

        blinkin.set(color);
    }

}