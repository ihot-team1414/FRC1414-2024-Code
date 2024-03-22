package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    private final TalonFX intakeMotor1;
    private final TalonFX intakeMotor2;
    private final TimeOfFlight intakeSensor;
    private Follower followerControl;
    private DutyCycleOut dutyCycleOutControl;
    private TalonFXConfiguration intakeMotorConfiguration;

    public IntakeSubsystem() {
        /*
         * Initialize CAN IDs.
         */

        intakeMotor1 = new TalonFX(IntakeConstants.kIntakeMotor1CanId);
        intakeMotor2 = new TalonFX(IntakeConstants.kIntakeMotor2CanId);
        intakeSensor = new TimeOfFlight(IntakeConstants.kIntakeSensorCandId);

        /*
         * Configure motor current limit.
         */
        intakeMotorConfiguration = new TalonFXConfiguration();
        intakeMotorConfiguration.withSlot0(IntakeConstants.kIntakeConfiguration);
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 10;

        intakeMotor1.getConfigurator().apply(intakeMotorConfiguration);
        intakeMotor2.getConfigurator().apply(intakeMotorConfiguration);

        /*
         * Configure motor neutral modes.
         */
        setDesiredNeutralMode();

        /*
         * Configure sensor.
         */
        intakeSensor.setRangingMode(RangingMode.Short, 24);

        /*
         * Set default controls.
         */
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

    /*
     * Control Methods
     */
    public void setDesiredNeutralMode() {
        intakeMotor1.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setDutyCycle(double dutyCycle) {
        intakeMotor1.setControl(dutyCycleOutControl.withOutput(dutyCycle));
    }

    public void stop() {
        intakeMotor1.stopMotor();
    }

    /*
     * Sensor Methods
     */
    public boolean isLoaded() {
        double distance = intakeSensor.getRange();
        return intakeSensor.isRangeValid() ? distance < IntakeConstants.kIndexThreshold : false;
    }

    @Override
    public void periodic() {
    }
}
