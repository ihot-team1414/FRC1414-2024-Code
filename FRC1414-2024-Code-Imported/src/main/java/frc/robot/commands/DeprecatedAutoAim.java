package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterData;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.ShooterDataUtils;

public class DeprecatedAutoAim extends Command {
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();

    private double fallbackDistance;

    public DeprecatedAutoAim() {
        addRequirements(pivot);
        this.fallbackDistance = 4.5;
    }

    public DeprecatedAutoAim(double fallbackDistance) {
        addRequirements(pivot);
        this.fallbackDistance = fallbackDistance;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double distance = VisionSubsystem.getInstance().getDistance().orElse(fallbackDistance);
        SmartDashboard.putNumber("Retrieved Distance", distance);
        pivot.setPosition(
                ShooterDataUtils.getInterpolatedEntry(ShooterData.fallbackSpeakerData, distance).getPosition());
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setPosition(Constants.PivotConstants.kStowPosition);
    }
}