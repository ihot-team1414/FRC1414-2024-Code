package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.LimelightHelpers;
import frc.utils.ShooterData;

public class AutoShoot extends Command {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private double fbDistance;

    public AutoShoot(double fbDistance) {
        addRequirements(drivetrain, intake, pivot, shooter);
        this.fbDistance = fbDistance;

    }

    @Override
    public void execute() {

        Rotation2d rotation = Rotation2d.fromRadians(0);

        if (LimelightHelpers.getTV("limelight-front")) {
            double yawError = LimelightHelpers.getTX("limelight-front");

            if (Math.abs(yawError) > Constants.DriveConstants.kAutoAimTeleopErrorMargin) {
                rotation = Rotation2d.fromDegrees(-yawError * Constants.DriveConstants.kAutoAimP);
            }

            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front");
            double distance = tagPose.getTranslation().getNorm();

            pivot.setPosition(ShooterData.getInstance().getShooterPosition(distance));
            shooter.setDutyCycle(ShooterData.getInstance().getShooterDutyCycle(distance));

            if (yawError < Constants.DriveConstants.kAutoAimTeleopErrorMargin
                    && pivot.isAtPositionSetpoint(ShooterData.getInstance().getShooterPosition(distance))
                    && shooter.isWithinVelocitylerance(ShooterData.getInstance().getMinShotVelocity(distance))) {
                intake.setDutyCycle(IntakeConstants.kSpeakerFeedDutyCycle);
            } else {
                intake.stop();
            }

            SmartDashboard.putNumber("Tag Distance", tagPose.getTranslation().getNorm());
        }

        else {
            pivot.setPosition(ShooterData.getInstance().getShooterPosition(fbDistance));
            shooter.setDutyCycle(ShooterData.getInstance().getShooterDutyCycle(fbDistance));

            // Check if rotation is correct?
            if (pivot.isAtPositionSetpoint(ShooterData.getInstance().getShooterPosition(fbDistance))
                    && shooter.isWithinVelocitylerance(ShooterData.getInstance().getMinShotVelocity(fbDistance))) {
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
    }
}