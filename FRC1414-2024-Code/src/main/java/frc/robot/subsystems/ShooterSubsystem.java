package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final Follower followerControl;
    private final VelocityDutyCycle velocityControl;
    private final DutyCycleOut dutyCycleOutControl;
    private final TalonFXConfiguration shooterMotorConfig;

    public ShooterSubsystem() {
        /*
         * Initialize CAN IDs.
         */
        shooterMotor1 = new TalonFX(ShooterConstants.kShooterMotor1CanId);
        shooterMotor2 = new TalonFX(ShooterConstants.kShooterMotor2CanId);

        /*
         * Configure motor current limit.
         */
        shooterMotorConfig = new TalonFXConfiguration();
        shooterMotorConfig.withSlot0(ShooterConstants.kShooterConfiguration);

        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;

        shooterMotor1.getConfigurator().apply(shooterMotorConfig);
        shooterMotor2.getConfigurator().apply(shooterMotorConfig);

        /*
         * Configure motor neutral modes.
         */
        setDesiredNeutralMode();

        /*
         * Set default controls.
         */
        velocityControl = new VelocityDutyCycle(0, 0, false, 0, 0, false, false, false);
        dutyCycleOutControl = new DutyCycleOut(0, true, false, false, false);
        followerControl = new Follower(shooterMotor1.getDeviceID(), true);

        shooterMotor2.setControl(followerControl);
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }

        return instance;
    }

    /*
     * Control Methods
     */
    public void setDesiredNeutralMode() {
        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setVoltage(Measure<Voltage> voltage) {
        shooterMotor1.set(voltage.in(Volts));
    }

    public void setVelocity(double velocity) {
        shooterMotor1.setControl(velocityControl.withVelocity(velocity));
    }

    public void setDutyCycle(double dutyCycle) {
        shooterMotor1.setControl(dutyCycleOutControl.withOutput(dutyCycle));
    }

    public void stop() {
        shooterMotor1.stopMotor();
    }

    /*
     * Sensor Methods
     */
    public boolean isWithinVelocitylerance(double target) {
        return shooterMotor1.getVelocity().getValueAsDouble() >= target;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", shooterMotor1.getVelocity().getValueAsDouble());
    }
}
