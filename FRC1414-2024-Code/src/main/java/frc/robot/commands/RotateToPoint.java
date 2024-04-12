package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.PIDConstants;

public class RotateToPoint extends Command {
    private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final PIDController rotController = new PIDController(5, 0, 0);

    private Translation2d target;

    public RotateToPoint(
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, 
            Translation2d target) {

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.target = target;

        addRequirements(drivetrainSubsystem);
    }

    public RotateToPoint(
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        Pose2d target) {
            this.translationXSupplier = translationXSupplier;
            this.translationYSupplier = translationYSupplier;
            this.target = target.getTranslation();
            
            addRequirements(drivetrainSubsystem);
        }

    @Override
    public void initialize(){
        rotController.setTolerance(1);
        rotController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {

        double x = drivetrainSubsystem.getCurrentPose().getX() - target.getX();
        double y = drivetrainSubsystem.getCurrentPose().getY() - target.getY();

        double angle = drivetrainSubsystem.getHeading().getDegrees();
        double additive = x < 0 ? 180 : 0;
        double difference = Math.toDegrees(Math.atan(y / x)) + additive;
        rotController.setSetpoint(difference);

        Rotation2d rotation = Rotation2d.fromDegrees(-rotController.calculate(-angle, rotController.getSetpoint()));

        double translationX = translationXSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;
        double translationY = translationYSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;

        drivetrainSubsystem
                .drive(new Transform2d(new Translation2d(translationX, translationY),
                        rotation),
                        true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.lock();
    }
}