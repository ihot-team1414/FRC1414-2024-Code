package frc.robot;

import java.util.TreeMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Routines;
import frc.robot.commands.ShooterPrimitives;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakePrimitives;
import frc.robot.commands.PivotPrimitives;
import frc.robot.commands.Align;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAimTeleop;
import frc.robot.commands.AutoRev;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootTeleop;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

        /*
         * Subsystems
         */
        private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
        private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

        /*
         * Controllers
         */
        PS5Controller driver = new PS5Controller(OIConstants.kDriverControllerPort);
        XboxController operator = new XboxController(OIConstants.kOperatorControllerPort);

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

                shooter.setDefaultCommand(ShooterPrimitives.rev(ShooterConstants.kShooterRestSpeed));
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
                new JoystickButton(driver, Button.kR1.value).onTrue(Routines.intake());
                new JoystickButton(driver, Button.kR2.value).whileTrue(Routines.eject());
                new JoystickButton(driver, Button.kL1.value).whileTrue(new AutoShootTeleop(
                                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> 0.9));

                new JoystickButton(driver, Button.kL2.value).whileTrue(new AutoAimTeleop(
                                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> 0.9));
        }

        private void configureOperator() {
                new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
                                .whileTrue(ShooterPrimitives.shoot()
                                                .alongWith(PivotPrimitives.pivotToPosition(
                                                                Constants.PivotConstants.kSpeakerShotPosition)));

                new JoystickButton(operator, XboxController.Button.kB.value)
                                .whileTrue(PivotPrimitives.pivotToPosition(Constants.PivotConstants.kStowPosition)
                                                .alongWith(new InstantCommand(
                                                                () -> ShooterSubsystem.getInstance().stop())));

                new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(
                                IntakePrimitives.speakerFeed().withTimeout(1)
                                                .finallyDo(() -> IntakeSubsystem.getInstance().stop()));

                new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileTrue(new AutoAimTeleop(
                                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> 0.9));

                new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(Routines.outtake());
        }

        public void configureAuto() {
                NamedCommands.registerCommand("Auto Aim", new AutoAim().repeatedly());
                NamedCommands.registerCommand("Auto Rev", new AutoRev().repeatedly());
                NamedCommands.registerCommand("Delayed Feed",
                                new WaitCommand(1).andThen(IntakePrimitives.speakerFeed().withTimeout(0.5)));
                NamedCommands.registerCommand("Delayed Feed 5 Note",
                                new WaitCommand(1).andThen(IntakePrimitives.speakerFeed().withTimeout(0.5)));
                NamedCommands.registerCommand("Warm Up", ShooterPrimitives.shoot());

                NamedCommands.registerCommand("Intake", Routines.intake());

                NamedCommands.registerCommand("Prepare",
                                ShooterPrimitives.rev(Constants.ShooterConstants.kSpeakerShotDutyCycle)
                                                .alongWith(PivotPrimitives.pivotToPosition(
                                                                Constants.PivotConstants.kSpeakerShotPosition)));

                NamedCommands.registerCommand("Stop Rev", new InstantCommand(
                                () -> ShooterSubsystem.getInstance().stop(), ShooterSubsystem.getInstance()));

                NamedCommands.registerCommand("Feed", IntakePrimitives.speakerFeed().withTimeout(0.75));

                NamedCommands.registerCommand("MFN Fallback 2", new AutoShoot(0).repeatedly());
                NamedCommands.registerCommand("MFN Fallback 3", new AutoShoot(0).repeatedly());
                NamedCommands.registerCommand("MFN Fallback 4 & 6", new AutoShoot(0).repeatedly());

                NamedCommands.registerCommand("W4N Fallback 2", new AutoShoot(0).repeatedly());
                NamedCommands.registerCommand("W4N Fallback 3 & 4", new AutoShoot(0).repeatedly());

                chooser.addOption("Four Note", AutoBuilder.buildAuto("Top Clear"));
                chooser.addOption("Five Note", AutoBuilder.buildAuto("Five Note"));
                chooser.addOption("Walton 3 Note", AutoBuilder.buildAuto("Walton"));
                chooser.addOption("Four Note Weak Side", AutoBuilder.buildAuto("Weak Side 4 Note"));
                chooser.addOption("Test", AutoBuilder.buildAuto("Test"));
                SmartDashboard.putData("Auto Chooser", this.chooser);

        }

        public Pose2d getStart(String auto) {
                return autoPoses.get(auto);
        }

        public Command getAutonomousCommand() {
                return chooser.getSelected();
        }
}