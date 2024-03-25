package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.LEDSubsystem;

public class LEDPrimitives {

    private static LEDSubsystem LED = LEDSubsystem.getInstance();

    public static Command LEDOn(double colorChange) {
        return new RunCommand(() -> LED.SetLED(colorChange));
    }
}
