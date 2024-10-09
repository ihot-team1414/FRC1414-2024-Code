package frc.robot.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class TestShooter extends Command {
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private Measure<Voltage> voltage = Volts.of(12);

    public TestShooter() {

        addRequirements(pivot, shooter);
    }

    @Override
    public void execute() {
        shooter.setVoltage(voltage);

        SmartDashboard.putNumber("Aim Shooter Distance", DrivetrainSubsystem.getInstance()
                .getDistanceToPoint(FieldConstants.allianceSpeakerPositionSupplier.get()));
        SmartDashboard.putNumber("Current Shooter Position", pivot.getPosition());

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}