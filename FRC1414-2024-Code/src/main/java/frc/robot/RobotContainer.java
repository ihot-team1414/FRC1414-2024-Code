package frc.robot;

import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Routines;
import frc.robot.commands.ShooterPrimitives;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
                new Drive(() -> MathUtil.applyDeadband(-driver.getLeftY(), Constants.OIConstants.kJoystickDeadband),
                        () -> MathUtil.applyDeadband(-driver.getLeftX(), Constants.OIConstants.kJoystickDeadband),
                        () -> MathUtil.applyDeadband(-driver.getRightX(), Constants.OIConstants.kJoystickDeadband),
                        () -> 0.9));
    }

    private void configureDriver() {
        new JoystickButton(driver, Button.kOptions.value)
                .onTrue(new InstantCommand(() -> {
                    DrivetrainSubsystem.getInstance().resetHeading();
                    DrivetrainSubsystem.getInstance()
                            .resetOdometry(new Pose2d(0, 0, DrivetrainSubsystem.getInstance().getHeading()));
                }));
        // new JoystickButton(driver, Button.kTriangle.value)
        // .whileTrue(SysId.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // new JoystickButton(driver,
        // Button.kSquare.value).whileTrue(SysId.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // new JoystickButton(driver, Button.kCross.value)
        // .whileTrue(SysId.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // new JoystickButton(driver, Button.kCircle.value)
        // .whileTrue(SysId.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        new JoystickButton(driver,
                Button.kTriangle.value).whileTrue(Routines.scoreAmp());
        new JoystickButton(driver,
                Button.kSquare.value).whileTrue(Routines.primeAmp());
        new JoystickButton(driver, Button.kR1.value).whileTrue(Routines.intake());
        new JoystickButton(driver, Button.kR2.value).onTrue(Routines.eject());
        new JoystickButton(driver, Button.kL1.value).whileTrue(new AutoShoot(
                () -> MathUtil.applyDeadband(-driver.getLeftY(),
                        Constants.OIConstants.kJoystickDeadband),
                () -> MathUtil.applyDeadband(-driver.getLeftX(),
                        Constants.OIConstants.kJoystickDeadband),
                () -> 0.9));

        new JoystickButton(driver, Button.kL2.value).whileTrue(ShooterPrimitives.warmUp());
    }

    private void configureOperator() {
    }

    public void configureAuto(){
        //autoPoses.put("Simple", new Pose2d(2, 7, Rotation2d.fromDegrees(180)));

        NamedCommands.registerCommand("Intake", Routines.intake());
        NamedCommands.registerCommand("Speaker", Routines.scoreSpeaker());
        NamedCommands.registerCommand("Warm Up", ShooterPrimitives.warmUp());

        chooser.setDefaultOption("Simple (4)", AutoBuilder.buildAuto("Simple (4)"));
        chooser.addOption("Center", AutoBuilder.buildAuto("Center"));
        chooser.addOption("Top Clear", AutoBuilder.buildAuto("Top Clear"));
        SmartDashboard.putData("Auto Chooser", this.chooser);
    }
    
    /*
    public void addToChoser(String name){
        chooser.addOption(name, findPoseToAuto(name));
    }*/

    public Pose2d getStart(String auto){
        return autoPoses.get(auto);
    }

    /*
    public SequentialCommandGroup findPoseToAuto(String auto){
        return new SequentialCommandGroup(AutoBuilder.pathfindToPose(getStart(auto), 
                                        new PathConstraints(6.5, 7, Units.degreesToRadians(540), Units.degreesToRadians(720))), 
                                        new PathPlannerAuto(auto));
    }*/

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}