package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class IntakePrimitives {
    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static LEDSubsystem led = LEDSubsystem.getInstance();
    private static PivotSubsystem pivot = PivotSubsystem.getInstance();

    public static Command intake() {
        return new InstantCommand(() -> led.setColor(0.03), led)
                .andThen(new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kIntakeDutyCycle),
                        intake))
                .until(() -> intake.isLoaded()).finallyDo(() -> {
                    intake.stop();
                    led.setColor(Constants.LEDConstants.kLEDBlack);
                });
    }

    public static Command speakerFeed() {
        return new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kSpeakerFeedDutyCycle), intake);
    }

    public static Command ampFeed() {
        return new RunCommand(() -> intake.setDutyCycle(Constants.IntakeConstants.kAmpFeedDutyCycle), intake);
    }

    public static Command ampFeedAtPivot(){
        return new RunCommand(() -> feedBeyondPosition(), intake);
    }

    private static void feedBeyondPosition(){
        if(pivot.isBeyondPosition(Constants.PivotConstants.kAmpBeyondThreshold)){
            intake.setDutyCycle(Constants.IntakeConstants.kAmpFeedDutyCycle);
        } 
    }
}
