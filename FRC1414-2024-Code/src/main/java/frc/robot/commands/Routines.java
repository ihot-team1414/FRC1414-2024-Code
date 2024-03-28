package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AmpConstants;
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
                                () -> AmpSubsystem.getInstance().setPosition(AmpConstants.kAmpScoringPosition)))))
                .repeatedly()
                .finallyDo(() -> {
                    amp.setPosition(AmpConstants.kAmpRestPosition);
                    pivot.setPosition(Constants.PivotConstants.kStowPosition);
                    RobotState.getInstance().setRobotConfiguration(RobotConfiguration.STOWED);
                });
    }

    public static Command scoreAmp() {
        return RobotState.transition(RobotConfiguration.AMP, ShooterPrimitives
                .rev(Constants.ShooterConstants.kAmpDutyCycleLeft)
                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kAmpScoringPosition)
                        .alongWith(new WaitCommand(0.2).andThen(new InstantCommand(
                                () -> AmpSubsystem.getInstance().setPosition(AmpConstants.kAmpScoringPosition)))))
                .andThen(IntakePrimitives.ampFeed()
                        .onlyIf(() -> pivot.getPosition() > Constants.PivotConstants.kAmpFeedPosition)
                        .repeatedly()))
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
                .andThen(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kSpeakerShotPosition)))
                .andThen(
                        RobotState.transition(RobotConfiguration.SHOOTING,
                                IntakePrimitives.speakerFeed().onlyIf(() -> shooter.isWithinVelocityTolerance(20))
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
        return PivotPrimitives.pivotToPosition(Constants.PivotConstants.kIntakePosition).andThen(
                IntakePrimitives.outtake())
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
