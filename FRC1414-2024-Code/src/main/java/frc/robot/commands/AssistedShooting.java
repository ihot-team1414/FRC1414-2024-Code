package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AssistedShooting extends Command {
    private final PivotSubsystem pivotSubsystem = PivotSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public AssistedShooting(){
        addRequirements(pivotSubsystem, shooterSubsystem);
    }


    @Override
    public void execute(){

        pivotSubsystem.setPivotAngle(pivotSubsystem.getPivotAngle());
    
    }

    @Override
    public void end(boolean interrupted){
        //pivotSubsystem.home();
    }
}
