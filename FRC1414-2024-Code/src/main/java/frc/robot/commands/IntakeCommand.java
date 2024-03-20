package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private PivotSubsystem pivot = PivotSubsystem.getInstance();

    public IntakeCommand() {
        addRequirements(intake, pivot);
    }

    @Override
    public void initialize() {
        intake.setDutyCycle(Constants.IntakeConstants.kIntakeDutyCycle);
    }
}
