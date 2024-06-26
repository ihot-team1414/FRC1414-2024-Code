package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LedDefault extends Command {
        private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
        private final LEDSubsystem leds = LEDSubsystem.getInstance();

        public LedDefault() {
                addRequirements(leds);
        }

        @Override
        public void execute() {
                if (shooter.isWithinVelocityTolerance(70)) {
                        leds.setColor(LEDConstants.kLEDGreenFlashing);
                } else if (intake.isNotePresent()) {
                        leds.setColor(LEDConstants.kLEDOrange);
                } else {
                        leds.setColor(LEDConstants.kLEDBlue);
                }

        }

        @Override
        public void end(boolean interrupted) {
                leds.setColor(LEDConstants.kLEDBlue);
        }
}