package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.utils.RobotState;
import frc.utils.RobotState.RobotConfiguration;

public class Robot extends TimedRobot {
    private Command autoCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        RobotState.getInstance().setRobotConfiguration(RobotConfiguration.DISABLED);
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        RobotState.getInstance().reset(true);

        autoCommand = robotContainer.getAutonomousCommand();

        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        RobotState.getInstance().reset(false);
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        RobotState.getInstance().reset(false);

        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
