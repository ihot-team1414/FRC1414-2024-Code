package frc.robot.commands;

import java.util.Optional;

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
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.LimelightHelpers;
import frc.utils.ShooterData;

public class AutoAim extends Command {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();

    private final PIDController alignmentController = new PIDController(DriveConstants.kAutoAimP,
            DriveConstants.kAutoAimI, DriveConstants.kAutoAimD);

    private double target;
    private double threshold = Constants.DriveConstants.kAutoAimAutoErrorMargin;

    public AutoAim() {
        addRequirements(drivetrain, pivot);
    }

    @Override
    public void initialize() {
        double yawError = VisionSubsystem.getInstance().getTX().orElse(0.0);
        double currentAngle = drivetrain.getHeading().getDegrees();

        target = yawError + currentAngle;
    }

    @Override
    public void execute() {

        Optional<Double> distance = VisionSubsystem.getInstance().getDistance();
        pivot.setPosition(ShooterData.getInstance().getShooterPosition(distance));

        double angle = drivetrain.getHeading().getDegrees();
        Rotation2d rotation = Rotation2d.fromDegrees(alignmentController.calculate(angle, target));

        drivetrain.drive(new Transform2d(new Translation2d(0, 0),
                rotation),
                true);
    }

    @Override
    public boolean isFinished() {
        Optional<Double> distance = VisionSubsystem.getInstance().getDistance();
        double pivotTarget = ShooterData.getInstance().getShooterPosition(distance);
        double error = drivetrain.getHeading().getDegrees() - target;

        return Math.abs(error) < threshold && pivot.isAtPositionSetpoint(pivotTarget);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.lock();
        pivot.setPosition(Constants.PivotConstants.kStowPosition);
    }
}