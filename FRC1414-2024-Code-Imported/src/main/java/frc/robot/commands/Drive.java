package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class Drive extends Command {
    private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier limitingFactorSupplier;

    public Drive(
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            DoubleSupplier limitingFactorSupplier) {

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.limitingFactorSupplier = limitingFactorSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double translationX = translationXSupplier.getAsDouble() * limitingFactorSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;
        double translationY = translationYSupplier.getAsDouble() * limitingFactorSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotation = rotationSupplier.getAsDouble() * limitingFactorSupplier.getAsDouble()
                * DriveConstants.kMaxAngularSpeed;

        drivetrainSubsystem
                .drive(new Transform2d(new Translation2d(translationX, translationY),
                        Rotation2d.fromRadians(rotation)),
                        true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.lock();
    }
}