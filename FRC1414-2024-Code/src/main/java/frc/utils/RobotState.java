package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotState {

    public enum RobotConfiguration {
        INDETERMINATE,
        STOWED,
        INTAKING,
        LIMELIGHT_SEARCHING,
        AIMING_SUCCESS,
        SHOOTING,
        AMP,
        EJECTING,
        DISABLED
    }

    private static RobotState instance;

    private boolean hasNote = false;
    private RobotConfiguration robotConfiguration = RobotConfiguration.STOWED;

    private RobotState() {
    }

    public static synchronized RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }

        return instance;
    }

    public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
        SmartDashboard.putBoolean("Game Piece?", this.hasNote);
    }

    public boolean hasNote() {
        SmartDashboard.putBoolean("Game Piece?", this.hasNote);
        return this.hasNote;
    }

    public void setRobotConfiguration(RobotConfiguration robotConfiguration) {
        SmartDashboard.putString("Robot Configuration", robotConfiguration.name());

        this.robotConfiguration = robotConfiguration;

    }

    public RobotConfiguration getRobotConfiguration() {
        SmartDashboard.putString("Robot Configuration", robotConfiguration.name());

        return robotConfiguration;
    }

    public static Command transition(RobotConfiguration to, Command command) {
        RobotState robotState = RobotState.getInstance();

        return command
                .alongWith(new WaitCommand(1).andThen(new InstantCommand(() -> robotState.setRobotConfiguration(to))));
    }

    public void reset(boolean hasNote) {
        setRobotConfiguration(RobotConfiguration.STOWED);
        setHasNote(hasNote);
    }
}