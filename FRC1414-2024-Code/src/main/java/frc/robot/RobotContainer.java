package frc.robot;

import java.util.Optional;
import java.util.TreeMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Routines;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakePrimitives;
import frc.robot.commands.PivotPrimitives;
import frc.robot.commands.PoseShoot;
import frc.robot.commands.Pass;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAimTeleop;
import frc.robot.commands.AutoRev;
import frc.robot.commands.AutoShootTeleop;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {

        /*
         * Subsystems
         */
        private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
        private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
        private final PivotSubsystem pivot = PivotSubsystem.getInstance();
        private final LEDSubsystem led = LEDSubsystem.getInstance();
        private final AmpSubsystem amp = AmpSubsystem.getInstance();
        private final VisionSubsystem vision = VisionSubsystem.getInstance();
        private Optional<Alliance> ds = DriverStation.getAlliance();

        /*
         * Controllers
         */
        PS5Controller driver = new PS5Controller(OIConstants.kDriverControllerPort);
        XboxController operator = new XboxController(OIConstants.kOperatorControllerPort);

        /*
         * Shooting, Auto and Passing
         */
        private TreeMap<String, Pose2d> autoPoses = new TreeMap<>();
        private SendableChooser<Command> chooser = new SendableChooser<>();
        private boolean isRed;
        private Translation2d target = new Translation2d();
        private Translation2d pass = new Translation2d();

        public RobotContainer() {

                isRed = ds.isPresent() && ds.get().equals(DriverStation.Alliance.Red);
                pass = isRed ? FieldConstants.getTagTranslation(5) : FieldConstants.getTagTranslation(6);
                target = isRed ? FieldConstants.kSector.get("RCenter") : FieldConstants.kSector.get("BCenter");
                

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
                                                () -> 1));

                amp.setDefaultCommand(new RunCommand(() -> amp.setPosition(AmpConstants.kAmpRestPosition), amp));
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
                                Button.kSquare.value).onTrue(Routines.primeAmp());
                new JoystickButton(driver, Button.kCircle.value).whileTrue(IntakePrimitives.speakerFeed());

                new JoystickButton(driver, Button.kR1.value).onTrue(Routines.intake());
                new JoystickButton(driver, Button.kR2.value).whileTrue(Routines.eject());

                new JoystickButton(driver, Button.kL1.value).whileTrue(new AutoShootTeleop(
                                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> 0.1));

                new JoystickButton(driver, Button.kL2.value).whileTrue(new AutoAimTeleop(
                                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> 1));

                new JoystickButton(driver, Button.kR3.value).whileTrue(new Pass(
                                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                                        Constants.OIConstants.kJoystickDeadband),
                                () -> MathUtil.applyDeadband(-driver.getLeftX(), 
                                        Constants.OIConstants.kJoystickDeadband), 
                                pass)
                );

                new POVButton(driver, 0).whileTrue(new RunCommand(() -> pivot.setPosition(pivot.getPosition() + 0.1), pivot));
                new POVButton(driver, 180).whileTrue(new RunCommand(() -> pivot.setPosition(pivot.getPosition() - 0.1), pivot));
                new POVButton(driver, 90).whileTrue(new PoseShoot(
                                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                                                Constants.OIConstants.kJoystickDeadband),
                                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                                Constants.OIConstants.kJoystickDeadband),
                                                 target, isRed));
        }

        private void configureOperator() {
                new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
                                .whileTrue(Routines.speakerShot());

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
                                () -> 1));

                new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(Routines.outtake());
                new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(Routines.trapShot());

        }

        public void configureAuto() {
                NamedCommands.registerCommand("Intake", IntakePrimitives.intake());
                NamedCommands.registerCommand("Auto Aim", new AutoAim().repeatedly());
                NamedCommands.registerCommand("Auto Aim 1", new AutoAim(1.75).repeatedly());
                NamedCommands.registerCommand("Auto Aim 2", new AutoAim(2.9).repeatedly());
                NamedCommands.registerCommand("Auto Aim 3", new AutoAim(3.5).repeatedly());
                NamedCommands.registerCommand("Auto Aim 4", new AutoAim(4.6).repeatedly());
                NamedCommands.registerCommand("Auto Aim 5", new AutoAim(4.6).repeatedly());
                NamedCommands.registerCommand("Auto Rev", new AutoRev().repeatedly());
                NamedCommands.registerCommand("Feed", IntakePrimitives.speakerFeed().withTimeout(0.75));
                chooser.addOption("Justin 5 Note Auto", AutoBuilder.buildAuto("Justin 5 Note Auto"));
                chooser.addOption("Justin 3 Note Auto", AutoBuilder.buildAuto("Justin 3 Note Auto"));
                chooser.addOption("2 Note Source", AutoBuilder.buildAuto("2 Note Source"));
                chooser.addOption("Beeline Auto", AutoBuilder.buildAuto("Beeline Auto"));
                SmartDashboard.putData("Auto Chooser", this.chooser);

        }

        public Pose2d getStart(String auto) {
                return autoPoses.get(auto);
        }

        public Command getAutonomousCommand() {
                return chooser.getSelected();
        }
}