package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class RotateToPose extends Command {
    private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private double angle;
    private Pose2d start;
    private Pose2d target;
    private Pose2d current;

    private final HolonomicDriveController holonomicDriveController;
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotController;

    public RotateToPose(
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            double angle) {

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.angle = angle;

        start = new Pose2d();
        target = new Pose2d();
        current = new Pose2d();
        xController = new PIDController(5, 0, 0);
        yController = new PIDController(5, 0, 0);
        rotController = new ProfiledPIDController(0.05, 0, 0, new TrapezoidProfile.Constraints(1, 1));
        holonomicDriveController = new HolonomicDriveController(xController, yController, rotController);
        holonomicDriveController.setTolerance(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0.5)));

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        start = drivetrainSubsystem.getCurrentPose();
        target = new Pose2d(start.getTranslation(), Rotation2d.fromDegrees(angle));

    }

    @Override
    public void execute() {
        current = drivetrainSubsystem.getCurrentPose();

        //SmartDashboard.putNumber("p angle", poseAngle);

        double translationX = translationXSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;
        double translationY = translationYSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;
/*
        drivetrainSubsystem
                .drive(new Transform2d(new Translation2d(translationX, translationY),
                        Rotation2d.fromDegrees(rotation)),
                        true);*/
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.lock();
    }
}