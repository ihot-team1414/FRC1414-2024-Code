package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePrimitives {
    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();

    public static Command intake() {
        return new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kIntakeDutyCycle),
                intake)
                .until(() -> intake.isLoaded()).finallyDo(() -> {
                    intake.stop();

                });
    }

    public static Command speakerFeed() {
        return new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kSpeakerFeedDutyCycle), intake);
    }

    public static Command ampFeed() {
        return new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kAmpFeedDutyCycle), intake);
    }

    public static Command outtake() {
        return new InstantCommand(() -> intake.setDutyCycle(-Constants.IntakeConstants.kIntakeDutyCycle),
                intake).repeatedly().finallyDo(() -> intake.stop());
    }
}
