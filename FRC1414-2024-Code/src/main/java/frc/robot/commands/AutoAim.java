package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.utils.LimelightHelpers;
import frc.utils.ShooterData;

public class AutoAim extends Command {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();
    private final PIDController drivetrainController = new PIDController(DriveConstants.kAutoAimP,
            DriveConstants.kAutoAimI, DriveConstants.kAutoAimD);

    public AutoAim() {
        addRequirements(drivetrain, pivot);
    }

    @Override
    public void execute() {

        Rotation2d rotation = Rotation2d.fromRadians(0);

        if (LimelightHelpers.getTV("limelight-front")) {
            double yawError = LimelightHelpers.getTX("limelight-front");

            if (Math.abs(yawError) > Constants.DriveConstants.kAutoAimAutoErrorMargin) {
                rotation = Rotation2d.fromDegrees(-drivetrainController.calculate(yawError, 0));
            }

            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front");
            double distance = tagPose.getTranslation().getNorm();

            pivot.setPosition(ShooterData.getInstance().getShooterPosition(distance));

            SmartDashboard.putNumber("Tag Distance", tagPose.getTranslation().getNorm());
        } else {
            pivot.setPosition(ShooterData.getInstance().getShooterPosition(0));
        }

        drivetrain
                .drive(new Transform2d(new Translation2d(0, 0),
                        rotation),
                        true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.lock();
        pivot.setPosition(Constants.PivotConstants.kStowPosition);
    }
}