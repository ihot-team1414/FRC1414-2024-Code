package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterData;
import frc.robot.commands.Routines;
import frc.robot.commands.TestShooter;
import frc.robot.commands.Drive;
import frc.robot.commands.LedDefault;
import frc.robot.commands.AimShooter;
import frc.robot.commands.DeprecatedAutoAim;
import frc.robot.commands.DeprecatedAutoRev;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
        private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
        private final PivotSubsystem pivot = PivotSubsystem.getInstance();
        private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
        private final DeflectorSubsystem deflector = DeflectorSubsystem.getInstance();
        private final LEDSubsystem leds = LEDSubsystem.getInstance();
        private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

        PS5Controller driver = new PS5Controller(OIConstants.kDriverControllerPort);
        XboxController operator = new XboxController(OIConstants.kOperatorControllerPort);

        private SendableChooser<Command> chooser = new SendableChooser<>();

        public RobotContainer() {
                initializeSubsystems();

                setDefaultCommands();
                configureAuto();
                configureDriver();
                configureOperator();
        }

        private void initializeSubsystems() {
                DrivetrainSubsystem.getInstance();
                PivotSubsystem.getInstance();
                IntakeSubsystem.getInstance();
                ShooterSubsystem.getInstance();
                DeflectorSubsystem.getInstance();

                LEDSubsystem.getInstance();
                VisionSubsystem.getInstance();
        }

        private void setDefaultCommands() {
                drivetrain.setDefaultCommand(
                                new Drive(() -> MathUtil.applyDeadband(DriverStation.getAlliance().orElse(
                                                DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                                ? -driver.getLeftY()
                                                                : driver.getLeftY(),
                                                Constants.OIConstants.kJoystickDeadband),
                                                () -> MathUtil.applyDeadband(DriverStation.getAlliance().orElse(
                                                                DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                                                ? -driver.getLeftX()
                                                                                : driver.getLeftX(),
                                                                Constants.OIConstants.kJoystickDeadband),
                                                () -> MathUtil.applyDeadband(-driver.getRightX(),
                                                                Constants.OIConstants.kJoystickDeadband),
                                                () -> 1));

                deflector.setDefaultCommand(deflector.stow().repeatedly());
                intake.setDefaultCommand(intake.autoLoad().repeatedly());
                pivot.setDefaultCommand(pivot.stow().repeatedly());
                leds.setDefaultCommand(new LedDefault());
        }

        private void configureDriver() {
                new JoystickButton(driver, PS5Controller.Button.kL1.value).whileTrue(intake.feed());
                new JoystickButton(driver, PS5Controller.Button.kR1.value).onTrue(Routines.intake());
                new JoystickButton(driver, PS5Controller.Button.kTriangle.value).whileTrue(Routines.ampAutoDrive());
                new JoystickButton(driver, PS5Controller.Button.kSquare.value).onTrue(Routines.ampMode());

                /*
                 * new POVButton(driver, 0)
                 * .whileTrue((new RunCommand(() -> pivot.setPosition(pivot.getPosition() +
                 * 0.075))));
                 * new POVButton(driver, 180)
                 * .whileTrue((new RunCommand(() -> pivot.setPosition(pivot.getPosition() -
                 * 0.01))));
                 * new POVButton(driver, 90).whileTrue(new TestShooter());
                 */
        }

        private void configureOperator() {
                new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(Routines.eject());
                new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(Routines.outtake());
                new JoystickButton(operator, XboxController.Button.kLeftBumper.value).whileTrue(Routines.shootMode(
                                () -> MathUtil.applyDeadband(DriverStation.getAlliance().orElse(
                                                DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                                ? -driver.getLeftY()
                                                                : driver.getLeftY(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> MathUtil.applyDeadband(DriverStation.getAlliance().orElse(
                                                DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                                ? -driver.getLeftX()
                                                                : driver.getLeftX(),
                                                Constants.OIConstants.kJoystickDeadband)));
                new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileTrue(Routines.passMode(
                                () -> MathUtil.applyDeadband(DriverStation.getAlliance().orElse(
                                                DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                                ? -driver.getLeftY()
                                                                : driver.getLeftY(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> MathUtil.applyDeadband(DriverStation.getAlliance().orElse(
                                                DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                                ? -driver.getLeftX()
                                                                : driver.getLeftX(),
                                                Constants.OIConstants.kJoystickDeadband)));
                new Trigger(() -> operator.getLeftTriggerAxis() > 0.5).whileTrue(Routines.subwooferShot());
                new Trigger(() -> operator.getRightTriggerAxis() > 0.5).whileTrue(Routines.reverseShot());

        }

        public void configureAuto() {
                NamedCommands.registerCommand("Intake", intake.intake());
                NamedCommands.registerCommand("Auto Aim", new DeprecatedAutoAim().repeatedly());
                NamedCommands.registerCommand("Odometry Aim",
                                new AimShooter(ShooterData.speakerData, () -> drivetrain.getDistanceToPoint(
                                                FieldConstants.allianceSpeakerPositionSupplier
                                                                .get()))
                                                .repeatedly());

                NamedCommands.registerCommand("Auto Aim 1", new DeprecatedAutoAim(1.75).repeatedly());
                NamedCommands.registerCommand("Auto Aim 2", new DeprecatedAutoAim(2.9).repeatedly());
                NamedCommands.registerCommand("Auto Aim 3", new DeprecatedAutoAim(3.5).repeatedly());
                NamedCommands.registerCommand("Auto Aim 4", new DeprecatedAutoAim(4.6).repeatedly());
                NamedCommands.registerCommand("Auto Aim 5", new DeprecatedAutoAim(4.6).repeatedly());
                NamedCommands.registerCommand("Auto Rev", new DeprecatedAutoRev().repeatedly());
                NamedCommands.registerCommand("Feed", intake.feed().withTimeout(0.75));
                NamedCommands.registerCommand("Infinite Feed", intake.feed());

                chooser.addOption("Justin 5 Note Auto", AutoBuilder.buildAuto("Justin 5 Note Auto"));
                chooser.addOption("Justin 3 Note Auto", AutoBuilder.buildAuto("Justin 3 Note Auto"));
                chooser.addOption("Center JR 4 Note Auto", AutoBuilder.buildAuto("JR 4 Note Center"));
                chooser.addOption("Shoot and Stay Amp Side", AutoBuilder.buildAuto("Shoot and Stay Amp Side"));
                chooser.addOption("3 Note Source (2,3)", AutoBuilder.buildAuto("3 Note Source (2,3)"));

                chooser.addOption("2 Note Source", AutoBuilder.buildAuto("2 Note Source"));
                chooser.addOption("Beeline Auto", AutoBuilder.buildAuto("Beeline Auto"));
                SmartDashboard.putData("Auto Chooser", this.chooser);
        }

        public Command getAutonomousCommand() {
                return chooser.getSelected();
        }
}