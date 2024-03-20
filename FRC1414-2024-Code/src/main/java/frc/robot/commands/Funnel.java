package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Funnel extends Command {

    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final PivotSubsystem pivotSubsystem = PivotSubsystem.getInstance();

    public Funnel(){
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        /* 
        if(drivetrainSubsystem.inWing() && shooterSubsystem.isWithinThreshold() && pivotSubsystem.isWithinThreshold()){
            intakeSubsystem.funnel();
        }*/

        intakeSubsystem.funnel();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }
}
