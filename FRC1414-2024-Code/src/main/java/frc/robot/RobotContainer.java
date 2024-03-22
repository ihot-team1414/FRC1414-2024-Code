package frc.robot;

import java.util.TreeMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Routines;
import frc.robot.commands.ShooterPrimitives;
import frc.robot.commands.Drive;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootTeleop;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    /*
     * Subsystems
     */
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

    /*
     * Controllers
     */
    PS5Controller driver = new PS5Controller(OIConstants.kDriverControllerPort);
    CommandXboxController operator = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /*
     * Auto Chooser
     */
    private TreeMap<String, Pose2d> autoPoses = new TreeMap<>();
    private SendableChooser<Command> chooser = new SendableChooser<>();

    public RobotContainer() {

        configureAuto();
        configureDriver();
        configureOperator();

        /*
         * We invert the controller axes to match the field coordinate system.
         * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-
         * system.html
         */
        DrivetrainSubsystem.getInstance().resetHeading();
        drivetrain.setDefaultCommand(
                new Drive(() -> MathUtil.applyDeadband(-driver.getLeftY(),
                        Constants.OIConstants.kJoystickDeadband),
                        () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                Constants.OIConstants.kJoystickDeadband),
                        () -> MathUtil.applyDeadband(-driver.getRightX(),
                                Constants.OIConstants.kJoystickDeadband),
                        () -> 0.9));
    }

    private void configureDriver() {
        new JoystickButton(driver, Button.kOptions.value)
                .onTrue(new InstantCommand(() -> {
                    DrivetrainSubsystem.getInstance().resetHeading();
                    DrivetrainSubsystem.getInstance()
                            .resetOdometry(new Pose2d(0, 0, DrivetrainSubsystem
                                    .getInstance().getHeading()));
                }));
        new JoystickButton(driver,
                Button.kTriangle.value).whileTrue(Routines.scoreAmp());
        new JoystickButton(driver,
                Button.kSquare.value).whileTrue(Routines.primeAmp());
        new JoystickButton(driver, Button.kR1.value).whileTrue(Routines.intake());
        new JoystickButton(driver, Button.kR2.value).whileTrue(Routines.eject());
        new JoystickButton(driver, Button.kL1.value).whileTrue(new AutoShootTeleop(
                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                        Constants.OIConstants.kJoystickDeadband),
                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                        Constants.OIConstants.kJoystickDeadband),
                () -> 0.9));

        new JoystickButton(driver, Button.kL2.value).whileTrue(ShooterPrimitives.warmUp());
    }

    private void configureOperator() {
    }

    public void configureAuto() {
        NamedCommands.registerCommand("Intake", Routines.intake());
        NamedCommands.registerCommand("Auto Shoot", new AutoShoot(() -> 0.9));
        NamedCommands.registerCommand("Warm Up", ShooterPrimitives.warmUp());

        chooser.setDefaultOption("Simple (4)", AutoBuilder.buildAuto("Simple (4)"));
        chooser.addOption("Center", AutoBuilder.buildAuto("Center"));
        chooser.addOption("Top Clear", AutoBuilder.buildAuto("Top Clear"));
        chooser.addOption("Simple (3)", AutoBuilder.buildAuto("Simple (3)"));
        SmartDashboard.putData("Auto Chooser", this.chooser);
    }

    public Pose2d getStart(String auto) {
        return autoPoses.get(auto);
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}