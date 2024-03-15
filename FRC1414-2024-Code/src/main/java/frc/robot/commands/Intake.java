package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public Intake(){
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.index();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }
}
