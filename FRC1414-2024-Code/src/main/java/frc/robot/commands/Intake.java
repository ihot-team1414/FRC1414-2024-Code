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
        if (intakeSubsystem.isLoaded()) {
            end(false);
        } else if(!intakeSubsystem.isLoaded()){
            intakeSubsystem.intake();
        }
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }
}
