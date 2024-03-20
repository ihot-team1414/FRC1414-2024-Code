package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class Pivot extends Command {
    private final PivotSubsystem pivotSubsystem = PivotSubsystem.getInstance();
    private double position;

    public Pivot(double position){
        addRequirements(pivotSubsystem);
        this.position = position;
    }

    @Override
    public void execute(){
        pivotSubsystem.setPivotAngle(position);
    }

    @Override
    public void end(boolean interrupted){
        pivotSubsystem.home();
    }
}
