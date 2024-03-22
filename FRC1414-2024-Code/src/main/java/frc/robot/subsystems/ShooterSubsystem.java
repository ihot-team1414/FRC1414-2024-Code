package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.LimelightHelpers;
import frc.utils.ShooterData;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;
    private final PivotSubsystem pivot;
    private final IntakeSubsystem intake;

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

        pivot = PivotSubsystem.getInstance();
        intake = IntakeSubsystem.getInstance();
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
        shooterMotor2.setControl(followerControl.withMasterID(32));
        shooterMotor1.setControl(dutyCycleOutControl.withOutput(dutyCycle));
    }

    public void setDutyCycle(double dutyCycleLeft, double dutyCycleRight) {
        shooterMotor1.setControl(dutyCycleOutControl.withOutput(dutyCycleLeft));
        shooterMotor2.setControl(dutyCycleOutControl.withOutput(-dutyCycleRight));
    }

    public void stop() {
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    public void turnToTarget() {
        Rotation2d rotation = Rotation2d.fromRadians(0);

        if (LimelightHelpers.getTV("limelight-front")) {
            double yawError = LimelightHelpers.getTX("limelight-front");

            if (Math.abs(yawError) > Constants.DriveConstants.kAutoAimErrorMargin) {
                rotation = Rotation2d.fromDegrees(-yawError * Constants.DriveConstants.kAutoAimP);
            }

            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front");
            double distance = tagPose.getTranslation().getNorm();

            pivot.setPosition(ShooterData.getInstance().getShooterPosition(distance));
            instance.setDutyCycle(ShooterData.getInstance().getShooterDutyCycle(distance));

            if (yawError < Constants.DriveConstants.kAutoAimErrorMargin
                    && pivot.isAtPositionSetpoint()
                    && instance.isWithinVelocitylerance(ShooterData.getInstance().getMinShotVelocity(distance))) {
                intake.setDutyCycle(IntakeConstants.kSpeakerFeedDutyCycle);
            } else {
                intake.stop();
            }
        }
        DrivetrainSubsystem.getInstance().drive(new Transform2d(new Translation2d(0, 0),
                rotation),
                true);
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
