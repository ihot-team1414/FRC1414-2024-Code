package frc.robot.commands;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ShooterSubsystem;

public class SysId {
    private static ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    private static SysIdRoutine sysId = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, (state) -> {
        SignalLogger.writeString("sysid-test-state", state.toString());
    }),
            new SysIdRoutine.Mechanism((voltage) -> shooter.setVoltage(voltage), null, shooter));

    public static Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public static Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}
