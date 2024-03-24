package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotPrimitives {
    private static PivotSubsystem pivot = PivotSubsystem.getInstance();

    public static Command pivotToPosition(double position) {
        return new RunCommand(() -> pivot.setPosition(position), pivot)
                .until(() -> pivot.isAtPositionSetpoint(position));
    }

    public static Command stow() {
        return pivotToPosition(Constants.PivotConstants.kStowPosition);
    }
}
