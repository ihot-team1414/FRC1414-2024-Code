package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AimDrive extends Command {
        private final DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();

        private final DoubleSupplier translationXSupplier;
        private final DoubleSupplier translationYSupplier;

        private Translation2d target = Constants.FieldConstants.bluePassPosition;

        private final PIDController rotController = new PIDController(12, 0.01, 0);

        public AimDrive(
                        DoubleSupplier translationXSupplier,
                        DoubleSupplier translationYSupplier,
                        Supplier<Translation2d> target) {

                this.translationXSupplier = translationXSupplier;
                this.translationYSupplier = translationYSupplier;
                this.target = target.get();

                rotController.setTolerance(1.2);
                rotController.enableContinuousInput(-180, 180);

                addRequirements(drive);
        }

        @Override
        public void execute() {
                double xPose = drive.getCurrentPose().getX();
                double yPose = drive.getCurrentPose().getY();

                double xAngle = xPose - target.getX();
                double yAngle = yPose - target.getY();

                double currentAngle = drive.getCurrentPose().getRotation().getDegrees();

                double fortran = DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? 180 : 0;

                double difference = Math.toDegrees(Math.atan(yAngle / xAngle)) + fortran;

                Rotation2d rotation = Rotation2d
                                .fromDegrees(rotController.calculate(currentAngle, difference));

                double translationX = translationXSupplier.getAsDouble()
                                * DriveConstants.kMaxSpeedMetersPerSecond;
                double translationY = translationYSupplier.getAsDouble()
                                * DriveConstants.kMaxSpeedMetersPerSecond;

                drive.drive(new Transform2d(new Translation2d(translationX, translationY),
                                rotation),
                                true);

        }

        @Override
        public void end(boolean interrupted) {
                drive.lock();
        }
}