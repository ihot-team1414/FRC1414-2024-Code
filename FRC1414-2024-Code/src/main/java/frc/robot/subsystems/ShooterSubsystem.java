package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final Follower followerControl;

    private final TalonFXConfiguration shooterMotorConfig;

    private final VelocityVoltage velocityControl;
    private final DutyCycleOut dutyCycleOutControl;
    private final VoltageOut voltageOutControl;
    private final VoltageOut voltageOutFOCControl;

    public ShooterSubsystem() {
        shooterMotor1 = new TalonFX(ShooterConstants.kShooterMotor1CanId);
        shooterMotor2 = new TalonFX(ShooterConstants.kShooterMotor2CanId);

        shooterMotorConfig = new TalonFXConfiguration();
        shooterMotorConfig.withSlot0(ShooterConstants.kShooterConfiguration);

        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;

        shooterMotor1.getConfigurator().apply(shooterMotorConfig);
        shooterMotor2.getConfigurator().apply(shooterMotorConfig);

        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor2.setNeutralMode(NeutralModeValue.Coast);

        velocityControl = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
        voltageOutControl = new VoltageOut(0, false, false, false, false);
        voltageOutFOCControl = new VoltageOut(0, true, false, false, false);
        dutyCycleOutControl = new DutyCycleOut(0, false, false, false, false);
        followerControl = new Follower(shooterMotor1.getDeviceID(), true);
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }

        return instance;
    }

    public void setVoltage(Measure<Voltage> voltage) {
        shooterMotor2.setControl(followerControl.withMasterID(32));
        shooterMotor1.setControl(voltageOutControl.withOutput(voltage.magnitude()));
    }

    public void setVoltageFOC(Measure<Voltage> voltage) {
        shooterMotor2.setControl(followerControl.withMasterID(32));
        shooterMotor1.setControl(voltageOutFOCControl.withOutput(voltage.magnitude()));
    }

    public void setDutyCycle(double dutyCycle) {
        shooterMotor2.setControl(followerControl.withMasterID(32));
        shooterMotor1.setControl(dutyCycleOutControl.withOutput(dutyCycle));
    }

    public void setVelocity(double velocity) {
        shooterMotor2.setControl(followerControl.withMasterID(32));
        shooterMotor1.setControl(velocityControl.withVelocity(velocity));
    }

    public void setVelocity(double velocityRight, double velocityLeft) {
        shooterMotor2.setControl(velocityControl.withVelocity(-velocityRight));
        shooterMotor1.setControl(velocityControl.withVelocity(velocityLeft));
    }

    public void stop() {
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    public boolean isWithinVelocityTolerance(double target) {
        return shooterMotor1.getVelocity().getValueAsDouble() >= target;
    }

    // Commands

    public Command rev(Measure<Voltage> voltage) {
        return rev(voltage, false);
    }

    public Command rev(Measure<Voltage> voltage, boolean useFOC) {
        if (useFOC) {
            return this.runEnd(() -> setVoltageFOC(voltage), () -> stop());
        }
        return this.runEnd(() -> setVoltage(voltage), () -> stop());
    }

    public Command rev(double dutyCycle) {
        return this.runEnd(() -> setDutyCycle(dutyCycle), () -> stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", shooterMotor1.getVelocity().getValueAsDouble());
    }

}
