package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class AutoShoot extends Command {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private double fallbackDistance;
    private double target;

    private final PIDController alignmentController = new PIDController(DriveConstants.kAutoAimP,
            DriveConstants.kAutoAimI, DriveConstants.kAutoAimD);

    public AutoShoot(double fallbackDistance) {
        addRequirements(drivetrain, intake, pivot, shooter);
        this.fallbackDistance = fallbackDistance;
    }

    @Override
    public void initialize() {
        double yawError = VisionSubsystem.getInstance().getTX().orElse(0.0);
        double currentAngle = drivetrain.getHeading().getDegrees();

        target = yawError + currentAngle;
    }

    @Override
    public void execute() {

        Rotation2d rotation = Rotation2d.fromRadians(0);

        double currentAngle = drivetrain.getHeading().getDegrees();

        rotation = Rotation2d
                .fromDegrees(alignmentController.calculate(drivetrain.getHeading().getDegrees(), target));

        if (VisionSubsystem.getInstance().getDistance().isPresent()) {
            RobotState.getInstance().setRobotConfiguration(RobotConfiguration.AIMING_SUCCESS);

            Optional<Double> distance = VisionSubsystem.getInstance().getDistance();

            pivot.setPosition(ShooterData.getInstance().getShooterPosition(distance));
            shooter.setVelocity(ShooterConstants.kShotSpeed);

            if (Math.abs(currentAngle - target) < Constants.DriveConstants.kAutoAimAutoErrorMargin
                    && pivot.isAtPositionSetpoint(ShooterData.getInstance().getShooterPosition(distance))
                    && shooter.isWithinVelocityTolerance(ShooterConstants.kShotSpeed)) {
                intake.setDutyCycle(IntakeConstants.kSpeakerFeedDutyCycle);
            } else {
                intake.stop();
            }
        } else {
            RobotState.getInstance().setRobotConfiguration(RobotConfiguration.LIMELIGHT_SEARCHING);

            pivot.setPosition(ShooterData.getInstance().getShooterPosition(fallbackDistance));
            shooter.setVelocity(ShooterConstants.kShotSpeed);

            // Check if rotation is correct?
            if (pivot.isAtPositionSetpoint(ShooterData.getInstance().getShooterPosition(fallbackDistance))
                    && shooter
                            .isWithinVelocityTolerance(
                                    ShooterConstants.kShotSpeed)) {
                RobotState.getInstance().setRobotConfiguration(RobotConfiguration.SHOOTING);
                intake.setDutyCycle(IntakeConstants.kSpeakerFeedDutyCycle);
            } else {
                intake.stop();
            }
        }

        drivetrain
                .drive(new Transform2d(new Translation2d(0, 0),
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