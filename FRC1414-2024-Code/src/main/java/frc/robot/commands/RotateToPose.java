package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class RotateToPose extends Command {
    private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final Translation2d target;
    private final PIDController rotController;
    private double additive;

    public RotateToPose(
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            Translation2d target) {

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.target = target;
        additive = 0;
        rotController = new PIDController(0.01, 0, 0);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {


        double robotToTargetX = drivetrainSubsystem.getCurrentPose().getX() - target.getX();
        double robotToTargetY = drivetrainSubsystem.getCurrentPose().getY() - target.getY();
        additive = robotToTargetX < 0 ? 180 : 0;
        double poseAngle = Math.toDegrees(Math.atan(robotToTargetY / robotToTargetX)) + additive;
        rotController.setSetpoint(poseAngle);
        double rotation = rotController.calculate(-(MathUtil.inputModulus(drivetrainSubsystem.getDegrees(), -180, 180)), rotController.getSetpoint());

        SmartDashboard.putNumber("p angle", poseAngle);

        double translationX = translationXSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;
        double translationY = translationYSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;

        drivetrainSubsystem
                .drive(new Transform2d(new Translation2d(translationX, translationY),
                        Rotation2d.fromDegrees(rotation)),
                        true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.lock();
    }
}