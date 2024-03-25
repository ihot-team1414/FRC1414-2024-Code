package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.LimelightHelpers;
import frc.utils.ShooterData;

public class AutoAimTeleop extends Command {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier limitingFactorSupplier;

    private double target;

    private final PIDController alignmentController = new PIDController(DriveConstants.kAutoAimP,
            DriveConstants.kAutoAimI, DriveConstants.kAutoAimD);

    public AutoAimTeleop(
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier limitingFactorSupplier) {

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.limitingFactorSupplier = limitingFactorSupplier;
        addRequirements(drivetrain, pivot, shooter);
    }

    @Override
    public void initialize() {
        double yawError = VisionSubsystem.getInstance().getTX().orElse(0.0);
        double currentAngle = drivetrain.getHeading().getDegrees();

        target = yawError + currentAngle;
    }

    @Override
    public void execute() {

        double translationX = translationXSupplier.getAsDouble() * limitingFactorSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;
        double translationY = translationYSupplier.getAsDouble() * limitingFactorSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;

        Optional<Double> distance = VisionSubsystem.getInstance().getDistance();
        pivot.setPosition(ShooterData.getInstance().getShooterPosition(distance));

        double angle = drivetrain.getHeading().getDegrees();
        Rotation2d rotation = Rotation2d.fromDegrees(alignmentController.calculate(angle, target));

        drivetrain
                .drive(new Transform2d(new Translation2d(translationX, translationY),
                        rotation),
                        true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.lock();
        pivot.setPosition(Constants.PivotConstants.kStowPosition);
    }
}