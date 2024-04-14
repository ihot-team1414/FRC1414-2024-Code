package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Routines;
import frc.robot.commands.Drive;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private final DeflectorSubsystem deflector = DeflectorSubsystem.getInstance();

    PS5Controller driver = new PS5Controller(OIConstants.kDriverControllerPort);
    XboxController operator = new XboxController(OIConstants.kOperatorControllerPort);

    private SendableChooser<Command> chooser = new SendableChooser<>();

    public RobotContainer() {
        initializeSubsystems();

        setDefaultCommands();
        configureAuto();
        configureDriver();
        configureOperator();

        DrivetrainSubsystem.getInstance().resetHeading();
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
                new Drive(() -> MathUtil.applyDeadband(-driver.getLeftY(),
                        Constants.OIConstants.kJoystickDeadband),
                        () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                Constants.OIConstants.kJoystickDeadband),
                        () -> MathUtil.applyDeadband(-driver.getRightX(),
                                Constants.OIConstants.kJoystickDeadband),
                        () -> 1));

        deflector.setDefaultCommand(deflector.stow().repeatedly());
        intake.setDefaultCommand(intake.autoLoad().repeatedly());
        pivot.setDefaultCommand(pivot.stow().repeatedly());
    }

    private void configureDriver() {
        new JoystickButton(driver, PS5Controller.Button.kL1.value).whileTrue(intake.feed());
        new JoystickButton(driver, PS5Controller.Button.kR1.value).onTrue(Routines.intake());
        new JoystickButton(driver, PS5Controller.Button.kSquare.value).onTrue(Routines.ampMode());
    }

    private void configureOperator() {
        new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(Routines.eject());
        new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(Routines.outtake());
        new JoystickButton(operator, XboxController.Button.kLeftBumper.value).whileTrue(Routines.shootMode(
                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                        Constants.OIConstants.kJoystickDeadband),
                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                        Constants.OIConstants.kJoystickDeadband)));
        new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileTrue(Routines.passMode(
                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                        Constants.OIConstants.kJoystickDeadband),
                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                        Constants.OIConstants.kJoystickDeadband)));
        new Trigger(() -> operator.getLeftTriggerAxis() > 0.5).whileTrue(Routines.subwooferShot());
        new Trigger(() -> operator.getRightTriggerAxis() > 0.5).whileTrue(Routines.reverseShot());
    }

    public void configureAuto() {
        NamedCommands.registerCommand("Intake", intake.intake());
        NamedCommands.registerCommand("Auto Aim", new DeprecatedAutoAim().repeatedly());
        NamedCommands.registerCommand("Auto Aim 1", new DeprecatedAutoAim(1.75).repeatedly());
        NamedCommands.registerCommand("Auto Aim 2", new DeprecatedAutoAim(2.9).repeatedly());
        NamedCommands.registerCommand("Auto Aim 3", new DeprecatedAutoAim(3.5).repeatedly());
        NamedCommands.registerCommand("Auto Aim 4", new DeprecatedAutoAim(4.6).repeatedly());
        NamedCommands.registerCommand("Auto Aim 5", new DeprecatedAutoAim(4.6).repeatedly());
        NamedCommands.registerCommand("Auto Rev", new DeprecatedAutoRev().repeatedly());
        NamedCommands.registerCommand("Feed", intake.feed().until(() -> !intake.isNotePresent()));
        chooser.addOption("Justin 5 Note Auto", AutoBuilder.buildAuto("Justin 5 Note Auto"));
        chooser.addOption("Justin 3 Note Auto", AutoBuilder.buildAuto("Justin 3 Note Auto"));
        chooser.addOption("2 Note Source", AutoBuilder.buildAuto("2 Note Source"));
        chooser.addOption("Beeline Auto", AutoBuilder.buildAuto("Beeline Auto"));
        SmartDashboard.putData("Auto Chooser", this.chooser);
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}