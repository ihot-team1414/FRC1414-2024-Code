package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPrimitives {
    private static ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public static Command rev(double dutyCycle) {
        return new InstantCommand(() -> shooter.setDutyCycle(dutyCycle), shooter);
    }

    public static Command rev(double dutyCycleLeft, double dutyCycleRight) {
        return new InstantCommand(() -> shooter.setDutyCycle(dutyCycleLeft, dutyCycleRight), shooter);
    }

    public static Command shoot() {
        return new InstantCommand(() -> {
            shooter.setVelocity(ShooterConstants.kShotSpeed);
        });
    }

}
