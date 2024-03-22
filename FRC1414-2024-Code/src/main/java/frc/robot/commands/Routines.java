package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Routines {
    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static PivotSubsystem pivot = PivotSubsystem.getInstance();
    private static ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public static Command primeAmp() {
        return PivotPrimitives.pivotToPosition(Constants.PivotConstants.kAmpPrimePosition);
    }

    public static Command scoreAmp() {
        return ShooterPrimitives
                .rev(Constants.ShooterConstants.kAmpDutyCycleLeft, Constants.ShooterConstants.kAmpDutyCycleRight)
                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kAmpScoringPosition)
                        .alongWith(IntakePrimitives.ampFeed()
                                .onlyIf(() -> pivot.getPosition() > Constants.PivotConstants.kAmpFeedPosition)
                                .repeatedly()))
                .finallyDo(() -> {
                    intake.stop();
                    shooter.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                });
    }

    public static Command scoreSpeaker() {
        return ShooterPrimitives.aim().withTimeout(2.5).finallyDo(() -> {
            intake.stop();
            pivot.setPosition(Constants.PivotConstants.kStowPosition);
            shooter.stop();
        });
    }

    public static Command intake() {
        return PivotPrimitives.pivotToPosition(Constants.PivotConstants.kIntakePosition)
                .andThen(IntakePrimitives.intake()).andThen(PivotPrimitives.stow())
                .finallyDo(() -> {
                    intake.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                });
    }

    public static Command eject() {
        return ShooterPrimitives.rev(Constants.ShooterConstants.kEjectDutyCycle)
                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kEjectPosition))
                .andThen(IntakePrimitives.speakerFeed().withTimeout(1)).finallyDo(() -> {
                    intake.stop();
                    shooter.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                });
    }

}
