package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.ShooterData;

public class Routines {

    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static PivotSubsystem pivot = PivotSubsystem.getInstance();
    private static ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static LEDSubsystem LED = LEDSubsystem.getInstance();
    private static VisionSubsystem vision = VisionSubsystem.getInstance();

    public static Command primeAmp() {
        return PivotPrimitives.pivotToPosition(Constants.PivotConstants.kAmpPrimePosition);
    }

    public static Command scoreAmp() {
        return ShooterPrimitives
                .rev(Constants.ShooterConstants.kAmpDutyCycleLeft, Constants.ShooterConstants.kAmpDutyCycleRight)
                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kAmpScoringPosition))
                .andThen(IntakePrimitives.ampFeed()
                        .onlyIf(() -> pivot.getPosition() > Constants.PivotConstants.kAmpFeedPosition).repeatedly())
                .finallyDo(() -> {
                    intake.stop();
                    shooter.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    new InstantCommand(() -> {
                        LED.setColor(Constants.LEDConstants.kLEDBlue);
                    });
                });
    }

    public static Command speakerShot() {
        return ShooterPrimitives
                .rev(Constants.ShooterConstants.kSpeakerShotDutyCycle)
                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kSpeakerShotPosition))
                .andThen(new InstantCommand(() -> {
                    LED.setColor(Constants.LEDConstants.kLEDGreen);
                }))
                .andThen(
                        IntakePrimitives.speakerFeed().onlyIf(() -> shooter.isWithinVelocityTolerance(20)).repeatedly())
                .finallyDo(() -> {
                    intake.stop();
                    shooter.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    new InstantCommand(() -> {
                        LED.setColor(Constants.LEDConstants.kLEDBlue);
                    });
                });
    }

    public static Command intake() {
        return PivotPrimitives.pivotToPosition(Constants.PivotConstants.kIntakePosition)
                .andThen(IntakePrimitives.intake()).andThen(PivotPrimitives.stow())
                .finallyDo((interrupted) -> {
                    if (!interrupted) {
                        LED.setColor(Constants.LEDConstants.kLEDOrange);
                    }
                    intake.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                });
    }

    public static Command outtake() {
        return PivotPrimitives.pivotToPosition(Constants.PivotConstants.kIntakePosition).andThen(
                IntakePrimitives.outtake())
                .finallyDo(() -> {
                    intake.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    new InstantCommand(() -> {
                        LED.setColor(Constants.LEDConstants.kLEDBlue);
                    });
                });
    }

    public static Command eject() {
        return ShooterPrimitives.rev(Constants.ShooterConstants.kEjectDutyCycle)
                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kEjectPosition))
                .andThen(IntakePrimitives.speakerFeed().withTimeout(1)).finallyDo(() -> {
                    intake.stop();
                    shooter.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    new InstantCommand(() -> {
                        LED.setColor(Constants.LEDConstants.kLEDBlue);
                    });
                });
    }

}
