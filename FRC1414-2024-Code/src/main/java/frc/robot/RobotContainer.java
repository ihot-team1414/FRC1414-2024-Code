// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpScore;
import frc.robot.commands.Funnel;
import frc.robot.commands.Intake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Optional;
import java.util.TreeMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DrivetrainSubsystem m_robotDrive = DrivetrainSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  PS5Controller m_driverController = new PS5Controller(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private final Optional<Alliance> ds = DriverStation.getAlliance();
  private TreeMap<String, Pose2d> autoPoses = new TreeMap<>();
  private int[] targetID;
  PathConstraints constraints;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  // AUTO CHOOSER
  private SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    targetID = new int[2];

    if(ds.isPresent()){
      targetID[0] = ds.get().equals(Alliance.Red) ? FieldConstants.kRedSpeakerID : FieldConstants.kBlueSpeakerID;
      targetID[1] = ds.get().equals(Alliance.Red) ? FieldConstants.kRedAmpID : FieldConstants.kBlueAmpID;
    }
    autoPoses.put("Simple", new Pose2d(2, 7, Rotation2d.fromDegrees(180))); 
    constraints = new PathConstraints(
      3.0, 4.0, 
      Units.degreesToRadians(540), 
      Units.degreesToRadians(720));

    chooser.setDefaultOption("Simple", findPoseToAuto("Simple"));    
    SmartDashboard.putData("Auto Chooser", this.chooser);

    // Configure the button bindings
    configureButtonBindings();
  
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                getDriverLeftY(),
                getDriverLeftX(),
                getDriverRightX(),
                true),
            m_robotDrive));

    shooter.setDefaultCommand( 
      new RunCommand(() -> shooter.shoot(), 
      shooter));
    }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //DRIVER CONTROLS

    //Zero the heading
    new JoystickButton(m_driverController, Button.kOptions.value).onTrue(new InstantCommand( () -> m_robotDrive.zeroHeading() ));
    new JoystickButton(m_driverController, Button.kL1.value).whileTrue(new RunCommand(() -> m_robotDrive.aimToTarget(getDriverLeftY(), getDriverLeftX(), targetID[1])));

    //Aim while moving
    new JoystickButton(m_driverController, Button.kR1.value)
                      .whileTrue(new RunCommand(() -> m_robotDrive.aimToTarget(
                        getDriverLeftY(),
                        getDriverLeftX(),
                        targetID[0])));

    //Slow mode while moving
    new JoystickButton(m_driverController, Button.kR1.value).whileTrue(
                      new RunCommand(() -> m_robotDrive.slowMode(
                        getDriverLeftY(),
                        getDriverLeftX(),
                        getDriverRightX())));
                        
    //Cardinal directions
    new JoystickButton(m_driverController, Button.kCircle.value).whileTrue(new RunCommand(() -> lockToCardinal(90)));
    new JoystickButton(m_driverController, Button.kTriangle.value).whileTrue(new RunCommand(() -> lockToCardinal(180)));
    new JoystickButton(m_driverController, Button.kSquare.value).whileTrue(new RunCommand(() -> lockToCardinal(-90)));
    new JoystickButton(m_driverController, Button.kCross.value).whileTrue(new RunCommand(() -> lockToCardinal(0)));

    //new JoystickButton(m_driverController, Button.kL2.value).whileTrue(new RunCommand(() -> m_robotDrive.driveToPose(new Pose2d(new Translation2d(2, 7), new Rotation2d(180)))));
  
    new JoystickButton(m_operatorController, XboxController.Button.kA.value).whileTrue(new Funnel());
    new JoystickButton(m_operatorController, XboxController.Button.kB.value).whileTrue(new Intake());
    new JoystickButton(m_operatorController, XboxController.Button.kX.value).whileTrue(new AmpScore());
    new JoystickButton(m_operatorController, XboxController.Button.kY.value).whileTrue(new RunCommand(() -> ShooterSubsystem.getInstance().outtake()));
  }

  private void lockToCardinal(double goal){
    m_robotDrive.cardinalDirection(
                        getDriverLeftY(),
                        getDriverLeftX(),
                        goal);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public double getDriverLeftY(){
    return MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
  }

  public double getDriverLeftX(){
    return MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
  }

  public double getDriverRightX(){
    return -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
  }

  public Pose2d getStart(String auto){
    return autoPoses.get(auto);
  }

  public SequentialCommandGroup findPoseToAuto(String auto){
    return new SequentialCommandGroup(AutoBuilder.pathfindToPose(getStart(auto), constraints, 0), new PathPlannerAuto(auto));

  }

}   