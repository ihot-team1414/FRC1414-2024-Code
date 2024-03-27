package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.RobotState;
import frc.utils.ShooterData;
import frc.utils.RobotState.RobotConfiguration;

public class AutoAim extends Command {
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();

    public AutoAim() {
        addRequirements(pivot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        boolean seesTarget = VisionSubsystem.getInstance().getDistance().isPresent();

        RobotState.getInstance().setRobotConfiguration(
                seesTarget ? RobotConfiguration.AIMING_SUCCESS : RobotConfiguration.LIMELIGHT_SEARCHING);

        Optional<Double> distance = VisionSubsystem.getInstance().getDistance();
        pivot.setPosition(ShooterData.getInstance().getShooterPosition(distance));

    }

    /*
     * @Override
     * public boolean isFinished() {
     * Optional<Double> distance = VisionSubsystem.getInstance().getDistance();
     * double pivotTarget = ShooterData.getInstance().getShooterPosition(distance);
     * double error = drivetrain.getHeading().getDegrees() - target;
     * 
     * return Math.abs(error) < threshold &&
     * pivot.isAtPositionSetpoint(pivotTarget);
     * }
     */

    @Override
    public void end(boolean interrupted) {
        pivot.setPosition(Constants.PivotConstants.kStowPosition);
        RobotState.getInstance().setRobotConfiguration(RobotConfiguration.STOWED);
    }
}