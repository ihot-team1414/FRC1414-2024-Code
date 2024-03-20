package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public Shoot(){
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.shootVolt();;
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stop();
    }
}
