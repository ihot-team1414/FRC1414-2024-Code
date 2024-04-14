package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class DeprecatedDriveToPose extends Command {
    private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private Pose2d target;
    private Pose2d current;

    private final HolonomicDriveController holonomicDriveController;
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotController;

    public DeprecatedDriveToPose(
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            double angle) {

        target = new Pose2d();
        current = new Pose2d();

        xController = new PIDController(5, 0, 0);
        yController = new PIDController(5, 0, 0);

        rotController = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(2, 2));

        holonomicDriveController = new HolonomicDriveController(xController, yController, rotController);
        holonomicDriveController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(2)));

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        target = new Pose2d(1.85, 7.5, new Rotation2d(90)); // start.getTranslation(), angle

    }

    @Override
    public void execute() {
        current = drivetrainSubsystem.getCurrentPose();
        ChassisSpeeds chassisSpeeds = holonomicDriveController.calculate(current, target, 0, target.getRotation());
        SwerveModuleState[] swerveModuleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);
        drivetrainSubsystem.setModuleStates(swerveModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] swerveModuleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);
        drivetrainSubsystem.setModuleStates(swerveModuleStates);
        drivetrainSubsystem.lock();
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();
    }
}