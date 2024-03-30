package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.utils.RobotState;
import frc.utils.RobotState.RobotConfiguration;

public class IntakePrimitives {
    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static PivotSubsystem pivot = PivotSubsystem.getInstance();

    public static Command intake() {
        return RobotState.transition(RobotConfiguration.INTAKING,
                new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kIntakeDutyCycle),
                        intake)
                        .until(() -> intake.isLoaded()).andThen(new WaitCommand(0.06)).finallyDo(() -> {
                            intake.stop();
                        }));
    }

    public static Command intakeAuto() {
        return RobotState.transition(RobotConfiguration.INTAKING,
                new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kIntakeDutyCycle),
                        intake)
                        .until(() -> intake.isLoaded()).andThen(new WaitCommand(0.06)).finallyDo(() -> {
                            intake.stop();
                        }))
                .onlyIf(() -> !intake.isLoaded());
    }

    public static Command speakerFeed() {
        return new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kSpeakerFeedDutyCycle), intake)
                .onlyIf(() -> pivot.getPosition() >= PivotConstants.kMinFeedPosition);
    }

    public static Command ampFeed() {
        return new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kAmpFeedDutyCycle), intake)
                .onlyIf(() -> pivot.getPosition() >= PivotConstants.kMinFeedPosition);

    }

    public static Command outtake() {
        return RobotState.transition(RobotConfiguration.EJECTING,
                new InstantCommand(() -> intake.setDutyCycle(-Constants.IntakeConstants.kIntakeDutyCycle),
                        intake).repeatedly().finallyDo(() -> intake.stop()));
    }

    public static Command stop() {
        return new InstantCommand(() -> intake.stop());
    }

}
