package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPose extends Command {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private Supplier<Pose2d> target;
    private Pose2d tolerance;

    private final HolonomicDriveController holonomicDriveController;
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotController;

    public DriveToPose(Supplier<Pose2d> target, Pose2d tolerance) {
        this.target = target;
        this.tolerance = tolerance;

        xController = new PIDController(5, 0, 0);
        yController = new PIDController(5, 0, 0);

        rotController = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(5, 5));

        holonomicDriveController = new HolonomicDriveController(xController, yController, rotController);
        holonomicDriveController.setTolerance(this.tolerance);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = holonomicDriveController.calculate(drivetrain.getCurrentPose(), target.get(),
                AutoConstants.kMaxSpeedMetersPerSecond, target.get().getRotation());
        SwerveModuleState[] swerveModuleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);
        drivetrain.setModuleStates(swerveModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.lock();
    }

    @Override
    public boolean isFinished() {
        System.out.println(drivetrain.getCurrentPose().getX());
        System.out.println(holonomicDriveController.atReference());
        return holonomicDriveController.atReference();
    }
}