package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {
    
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final PivotSubsystem pivotSubsystem = PivotSubsystem.getInstance();
    double pivot = 0;
    double speed = 0;

    public StartShooter(){
        addRequirements(shooterSubsystem, pivotSubsystem);
    }

    @Override
    public void execute(){
        
        pivot = pivotSubsystem.getPivotAngle();
        pivotSubsystem.setPivotAngle(pivot);
        
        speed = shooterSubsystem.getShooterSpeed();
        shooterSubsystem.setShooterSpeed(speed);
    }

    @Override
    public void end(boolean interrupted){

    }
}
