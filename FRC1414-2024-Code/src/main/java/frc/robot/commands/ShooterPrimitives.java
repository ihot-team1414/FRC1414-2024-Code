package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPrimitives {
    private static ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public static Command rev(double dutyCycle) {
        return new InstantCommand(() -> shooter.setDutyCycle(Constants.ShooterConstants.kEjectDutyCycle), shooter);
    }
}
