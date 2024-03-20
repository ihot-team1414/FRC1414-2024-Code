package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Outtake extends Command {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public Outtake(){
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.outtake();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }
}
