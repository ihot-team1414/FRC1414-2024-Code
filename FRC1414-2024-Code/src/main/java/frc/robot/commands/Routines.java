package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DeflectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterData;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Routines {
    private static DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();
    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static PivotSubsystem pivot = PivotSubsystem.getInstance();
    private static ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static DeflectorSubsystem deflector = DeflectorSubsystem.getInstance();

    public static Command intake() {
        return pivot.rotateToPosition(PivotConstants.kIntakePosition)
                .andThen(intake.intake());
    }

    public static Command ampMode() {
        return pivot.rotateToPosition(PivotConstants.kAmpScoringPosition).alongWith(new WaitCommand(0.2)
                .andThen(
                        shooter.rev(ShooterConstants.kAmpVoltage)
                                .alongWith(deflector.rotateToPosition(
                                        DeflectorConstants.kDeflectorScoringPosition))));
    }

    public static Command shootMode(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        Supplier<Translation2d> targetSupplier = () -> DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == Alliance.Blue
                        ? FieldConstants.getTagTranslation(FieldConstants.kBlueSpeakerID)
                        : FieldConstants.getTagTranslation(FieldConstants.kRedSpeakerID);

        return new AimDrive(translationXSupplier, translationYSupplier, targetSupplier)
                .alongWith(
                        new AimShooter(ShooterData.speakerData,
                                () -> drive.getDistanceToPoint(targetSupplier.get())));
    }

    public static Command passMode(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        Supplier<Translation2d> targetSupplier = () -> DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == Alliance.Blue
                        ? FieldConstants.bluePassPosition
                        : FieldConstants.redPassPosition;

        return new AimDrive(translationXSupplier, translationYSupplier, targetSupplier)
                .alongWith(
                        new AimShooter(ShooterData.passingData,
                                () -> drive.getDistanceToPoint(targetSupplier.get())));
    }

    public static Command outtake() {
        return pivot.rotateToPosition(PivotConstants.kOuttakePosition)
                .andThen(intake.outtake().alongWith(shooter.rev(ShooterConstants.kOuttakeVoltage)));
    }

    public static Command fixedShot(double pivotPosition, Measure<Voltage> shooterVoltage,
            double minimumShotVelocity) {
        return shooter.rev(shooterVoltage).alongWith(pivot.rotateToPosition(pivotPosition).andThen(
                intake.feed().onlyIf(() -> shooter.isWithinVelocityTolerance(minimumShotVelocity))
                        .repeatedly()));
    }

    public static Command subwooferShot() {
        return fixedShot(PivotConstants.kSubwooferShotPosition, ShooterConstants.kSubwooferShotVoltage,
                ShooterConstants.kSubwooferShotVelocity);
    }

    public static Command eject() {
        return fixedShot(PivotConstants.kEjectPosition, ShooterConstants.kEjectVoltage,
                ShooterConstants.kEjectVelocity);
    }

    public static Command reverseShot() {
        return fixedShot(PivotConstants.kReverseShotPosition, ShooterConstants.kSubwooferShotVoltage,
                ShooterConstants.kSubwooferShotVelocity);
    }
}
