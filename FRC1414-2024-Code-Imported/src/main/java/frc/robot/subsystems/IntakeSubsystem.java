package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    private final TalonFX intakeMotor1;
    private final TalonFX intakeMotor2;

    private final TimeOfFlight intakeSensorFront;
    private final TimeOfFlight intakeSensorMiddle;
    private final TimeOfFlight intakeSensorBack;

    private Follower followerControl;
    private DutyCycleOut dutyCycleOutControl;
    private TalonFXConfiguration intakeMotorConfiguration;

    private Debouncer presenceDebouncer = new Debouncer(0.1,
            Debouncer.DebounceType.kFalling);

    private Debouncer loadedDebouncer = new Debouncer(0.1,
            Debouncer.DebounceType.kFalling);

    public IntakeSubsystem() {
        intakeMotor1 = new TalonFX(IntakeConstants.kIntakeMotor1CanId);
        intakeMotor2 = new TalonFX(IntakeConstants.kIntakeMotor2CanId);

        intakeSensorFront = new TimeOfFlight(IntakeConstants.kIntakeSensorFrontCanId);
        intakeSensorMiddle = new TimeOfFlight(IntakeConstants.kIntakeSensorMiddleCanId);
        intakeSensorBack = new TimeOfFlight(IntakeConstants.kIntakeSensorBackCanId);

        intakeMotorConfiguration = new TalonFXConfiguration();
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kIntakeMotorCurrentLimit;

        intakeMotor1.getConfigurator().apply(intakeMotorConfiguration);
        intakeMotor2.getConfigurator().apply(intakeMotorConfiguration);

        intakeMotor1.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor2.setNeutralMode(NeutralModeValue.Brake);

        intakeSensorFront.setRangingMode(RangingMode.Short, 24);
        intakeSensorMiddle.setRangingMode(RangingMode.Short, 24);
        intakeSensorBack.setRangingMode(RangingMode.Short, 24);

        dutyCycleOutControl = new DutyCycleOut(0, true, false, false, false);
        followerControl = new Follower(intakeMotor1.getDeviceID(), true);
        intakeMotor2.setControl(followerControl);
    }

    public static synchronized IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    public void setDutyCycle(double dutyCycle) {
        intakeMotor1.setControl(dutyCycleOutControl.withOutput(dutyCycle));
    }

    public void stop() {
        intakeMotor1.stopMotor();
    }

    public boolean isFrontSensorTripped() {
        return false;
        // return intakeSensorFront.getRange() < IntakeConstants.kFrontSensorThreshold;
    }

    public boolean isMiddleSensorTripped() {
        // return intakeSensorMiddle.getRange() <
        // IntakeConstants.kMiddleSensorThreshold;
        return false;
    }

    public boolean isBackSensorTripped() {
        return intakeSensorBack.getRange() < IntakeConstants.kBackSensorThreshold;
    }

    public boolean isNotePresent() {
        boolean isNotePresent = isFrontSensorTripped()
                || isMiddleSensorTripped()
                || isBackSensorTripped();

        return presenceDebouncer.calculate(isNotePresent);
    }

    public boolean isLoaded() {
        return loadedDebouncer.calculate(intakeSensorBack.getRange() < IntakeConstants.kBackSensorThreshold);
    }

    // Commands
    public Command intake() {
        return this.runEnd(() -> setDutyCycle(IntakeConstants.kIntakeDutyCycle), () -> stop()).until(() -> isLoaded())
                .onlyIf(() -> !isLoaded());
    }

    public Command feed() {
        return this.runEnd(() -> setDutyCycle(IntakeConstants.kFeedDutyCycle), () -> stop());
    }

    public Command outtake() {
        return this.runEnd(() -> setDutyCycle(IntakeConstants.kOuttakeDutyCycle), () -> stop());
    }

    public Command autoLoad() {
        return intake().onlyIf(() -> isMiddleSensorTripped() || isFrontSensorTripped());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Back Sensor Distance", intakeSensorBack.getRange());
        SmartDashboard.putNumber("Middle Sensor Range", intakeSensorMiddle.getRange());
        SmartDashboard.putNumber("Front Sensor Range", intakeSensorFront.getRange());
        SmartDashboard.putBoolean("Note Present", isNotePresent());
    }
}
