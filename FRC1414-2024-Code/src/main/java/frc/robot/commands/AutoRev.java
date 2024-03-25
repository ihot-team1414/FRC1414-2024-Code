package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.ShooterData;

public class AutoRev extends Command {
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public AutoRev() {
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Optional<Double> distance = VisionSubsystem.getInstance().getDistance();

        shooter.setDutyCycle(ShooterData.getInstance().getShooterDutyCycle(distance));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}