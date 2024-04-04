package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.RobotState;
import frc.utils.RobotState.RobotConfiguration;

public class Routines {

    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static PivotSubsystem pivot = PivotSubsystem.getInstance();
    private static ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static AmpSubsystem amp = AmpSubsystem.getInstance();

    public static Command primeAmp() {
        return RobotState.transition(RobotConfiguration.AMP,
                PivotPrimitives.pivotToPosition(Constants.PivotConstants.kAmpScoringPosition)
                        .alongWith(new WaitCommand(0.2).andThen(new InstantCommand(
                                () -> AmpSubsystem.getInstance().setPosition(AmpConstants.kAmpScoringPosition),
                                AmpSubsystem.getInstance()))))
                .repeatedly()
                .finallyDo((interrupted) -> {
                    // if (!interrupted) {
                    // amp.setPosition(AmpConstants.kAmpRestPosition);
                    // pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    // RobotState.getInstance().setRobotConfiguration(RobotConfiguration.STOWED);
                    // }
                });
    }

    public static Command scoreAmp() {
        return RobotState.transition(RobotConfiguration.AMP, ShooterPrimitives
                .revVolt(Constants.ShooterConstants.kAmpDutyCycleLeft)
                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kAmpScoringPosition)
                        .alongWith(new WaitCommand(0.2).andThen(new InstantCommand(
                                () -> AmpSubsystem.getInstance().setPosition(AmpConstants.kAmpScoringPosition),
                                AmpSubsystem.getInstance()))))
                .andThen(IntakePrimitives.ampFeed()
                        .onlyIf(() -> pivot.getPosition() > Constants.PivotConstants.kAmpFeedPosition)
                        .repeatedly().alongWith(new InstantCommand(
                                () -> AmpSubsystem.getInstance().setPosition(AmpConstants.kAmpScoringPosition),
                                AmpSubsystem.getInstance()))))
                .finallyDo(() -> {
                    intake.stop();
                    shooter.stop();
                    amp.setPosition(AmpConstants.kAmpRestPosition);
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    RobotState.getInstance().setRobotConfiguration(RobotConfiguration.STOWED);
                });
    }

    public static Command speakerShot() {
        return RobotState.transition(RobotConfiguration.AIMING_SUCCESS, ShooterPrimitives
                .shoot()
                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kSpeakerShotPosition))
                .alongWith(ShooterPrimitives.rev(ShooterConstants.kSpeakerShotDutyCycle))).withTimeout(1)
                .andThen(
                        RobotState.transition(RobotConfiguration.SHOOTING,
                                IntakePrimitives.speakerFeed()
                                        .repeatedly()))
                .finallyDo(() -> {
                    intake.stop();
                    shooter.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    RobotState.getInstance().setRobotConfiguration(RobotConfiguration.STOWED);
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

    public static Command outtake() {
        return PivotPrimitives.pivotToPosition(Constants.PivotConstants.kIntakePosition)
                .onlyIf(() -> RobotState.getInstance().getRobotConfiguration() == RobotConfiguration.AMP).andThen(
                        IntakePrimitives.outtake().alongWith(ShooterPrimitives.rev(-0.3)))
                .finallyDo(() -> {
                    intake.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    RobotState.getInstance().setRobotConfiguration(RobotConfiguration.STOWED);
                });
    }

    public static Command eject() {
        return RobotState
                .transition(RobotConfiguration.EJECTING,
                        ShooterPrimitives.rev(Constants.ShooterConstants.kEjectDutyCycle)
                                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kEjectPosition))
                                .andThen(IntakePrimitives.speakerFeed().withTimeout(1)))
                .finallyDo(() -> {
                    intake.stop();
                    shooter.stop();
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    RobotState.getInstance().setRobotConfiguration(RobotConfiguration.STOWED);
                });
    }

}
