package frc.robot.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.ShooterDataUtils;
import frc.utils.ShooterEntry;

import static edu.wpi.first.units.Units.Volts;

import java.util.TreeMap;
import java.util.function.DoubleSupplier;

public class AimShooter extends Command {
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private TreeMap<Double, ShooterEntry> shooterData;
    private DoubleSupplier distanceSupplier;
    private Measure<Voltage> voltage = Volts.of(12);

    public AimShooter(
            TreeMap<Double, ShooterEntry> shooterData,
            DoubleSupplier distanceSupplier,
            Measure<Voltage> voltage) {

        this.shooterData = shooterData;
        this.distanceSupplier = distanceSupplier;
        this.voltage = voltage;
        addRequirements(pivot, shooter);
    }

    public AimShooter(
            TreeMap<Double, ShooterEntry> shooterData,
            DoubleSupplier distanceSupplier) {

        this.shooterData = shooterData;
        this.distanceSupplier = distanceSupplier;

        addRequirements(pivot, shooter);
    }

    @Override
    public void execute() {
        ShooterEntry shooterEntry = ShooterDataUtils.getInterpolatedEntry(shooterData, distanceSupplier.getAsDouble());
        pivot.setPosition(shooterEntry.getPosition());
        shooter.setVoltage(voltage);

        SmartDashboard.putNumber("Aim Shooter Distance", distanceSupplier.getAsDouble());
        SmartDashboard.putNumber("Aim Shooter Position", shooterEntry.getPosition());
        SmartDashboard.putNumber("Aim Shooter Voltage", this.voltage.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        pivot.setPosition(PivotConstants.kStowPosition);
    }
}