package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.RobotState;
import frc.utils.RobotState.RobotConfiguration;
import frc.utils.ShooterData;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AutoShootTeleop extends Command {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier limitingFactorSupplier;

    private double target;

    private final PIDController alignmentController = new PIDController(DriveConstants.kAutoAimP,
            DriveConstants.kAutoAimI, DriveConstants.kAutoAimD);

    public AutoShootTeleop(
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier limitingFactorSupplier) {

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.limitingFactorSupplier = limitingFactorSupplier;
        addRequirements(drivetrain, intake, pivot, shooter);
    }

    @Override
    public void initialize() {
        double yawError = VisionSubsystem.getInstance().getTX().orElse(0.0);
        double currentAngle = drivetrain.getHeading().getDegrees();

        target = -yawError;

        alignmentController.setTolerance(DriveConstants.kAutoAimTeleopErrorMargin);
        alignmentController.enableContinuousInput(-180, 180);
        SmartDashboard.putNumber("Turning P", DriveConstants.kAutoAimP);
        SmartDashboard.putNumber("Turning I", DriveConstants.kAutoAimI);
        SmartDashboard.putNumber("Turning D", DriveConstants.kAutoAimD);
        alignmentController.setSetpoint(target);

    }

    @Override
    public void execute() {
        alignmentController.setP(SmartDashboard.getNumber("Turning P", DriveConstants.kAutoAimP));
        alignmentController.setI(SmartDashboard.getNumber("Turning I", DriveConstants.kAutoAimI));
        alignmentController.setD(SmartDashboard.getNumber("Turning D", DriveConstants.kAutoAimD));

        target = -VisionSubsystem.getInstance().getTX().orElse(0.0);
        double angle = drivetrain.getHeading().getDegrees();

        boolean seesTarget = VisionSubsystem.getInstance().getDistance().isPresent();

        RobotState.getInstance().setRobotConfiguration(
                seesTarget ? RobotConfiguration.AIMING_SUCCESS : RobotConfiguration.LIMELIGHT_SEARCHING);

        Rotation2d rotation = Rotation2d.fromDegrees(-alignmentController.calculate(-angle, target));

        double translationX = translationXSupplier.getAsDouble() * limitingFactorSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;
        double translationY = translationYSupplier.getAsDouble() * limitingFactorSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;

        Optional<Double> distance = VisionSubsystem.getInstance().getDistance();

        pivot.setPosition(ShooterData.getInstance().getShooterPosition(distance));
        shooter.setVelocity(ShooterConstants.kShotSpeed);
        // shooter.setDutyCycle(ShooterConstants.kShotSpeedDutyCycle);
        // shooter.setVoltage(ShooterConstants.kShotSpeedDutyCycle);

        if (Math.abs(target + angle) < DriveConstants.kAutoAimTeleopErrorMargin
                && pivot.isAtPositionSetpoint(ShooterData.getInstance().getShooterPosition(distance))
                && shooter.isWithinVelocityTolerance(ShooterConstants.kShotSpeed)
        //
        ) {
            RobotState.getInstance().setRobotConfiguration(RobotConfiguration.SHOOTING);
            intake.setDutyCycle(IntakeConstants.kSpeakerFeedDutyCycle);
        } else {
            // intake.stop();
        }

        drivetrain
                .drive(new Transform2d(new Translation2d(translationX, translationY),
                        rotation),
                        true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.lock();
        intake.stop();
        shooter.stop();
        pivot.setPosition(Constants.PivotConstants.kStowPosition);
        RobotState.getInstance().setRobotConfiguration(RobotConfiguration.STOWED);
    }
}