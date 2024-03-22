package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;

    private final TalonFX pivotMotor1;
    private final TalonFX pivotMotor2;
    private final Follower followerControl;
    private final MotionMagicDutyCycle motionMagicControl;
    private final TalonFXConfiguration pivotMotorConfig;

    public PivotSubsystem() {
        /*
         * Initialize CAN IDs.
         */
        pivotMotor1 = new TalonFX(PivotConstants.kPivotMotor1CanId);
        pivotMotor2 = new TalonFX(PivotConstants.kPivotMotor2CanId);

        /*
         * Configure motion magic constants.
         */
        pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.withSlot0(PivotConstants.kPivotConfiguration);
        pivotMotorConfig.withMotionMagic(PivotConstants.kPivotMotionMagic);
        pivotMotor1.getConfigurator().apply(pivotMotorConfig);
        pivotMotor2.getConfigurator().apply(pivotMotorConfig);

        /*
         * Configure motor neutral modes.
         */
        setDesiredNeutralMode();

        /*
         * Set default controls.
         */
        motionMagicControl = new MotionMagicDutyCycle(0, true, 0, 0, false, false, false);
        followerControl = new Follower(pivotMotor1.getDeviceID(), true);
        pivotMotor2.setControl(followerControl);
    }

    public static synchronized PivotSubsystem getInstance() {
        if (instance == null) {
            instance = new PivotSubsystem();
        }
        return instance;
    }

    /*
     * Control Methods
     */
    public void setDesiredNeutralMode() {
        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setPosition(double position) {
        pivotMotor1.setControl(motionMagicControl.withPosition(position));
    }

    /*
     * Sensor Methods
     */
    public boolean isAtPositionSetpoint() {
        return Math.abs(pivotMotor1.getPosition().getValueAsDouble()
                - motionMagicControl.Position) < PivotConstants.kPivotErrorMargin;
    }

    public double getPosition() {
        return pivotMotor1.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", pivotMotor1.getPosition().getValueAsDouble());
    }
}
