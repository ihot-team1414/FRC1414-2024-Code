package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpScore extends Command {
    
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final PivotSubsystem pivotSubsystem = PivotSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    double pivot = 0;
    double speed = 0;

    public AmpScore(){
        addRequirements(shooterSubsystem, pivotSubsystem);
    }

    @Override
    public void execute(){
        pivotSubsystem.setPivotAngle(PivotConstants.kAmpAngle);
        shooterSubsystem.outtake();

        if(pivotSubsystem.isWithinThreshold() && shooterSubsystem.isWithinThreshold()){
            intakeSubsystem.funnel();
        }
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stop();
    }
}
