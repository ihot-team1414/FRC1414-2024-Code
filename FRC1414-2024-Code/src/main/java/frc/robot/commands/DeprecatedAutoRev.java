package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class DeprecatedAutoRev extends Command {
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public DeprecatedAutoRev() {
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVelocity(ShooterConstants.kShotSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}