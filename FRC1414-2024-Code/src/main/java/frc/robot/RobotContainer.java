// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimToTarget;
import frc.robot.commands.Drive;
import frc.robot.commands.Lock;
import frc.robot.commands.AimToTarget;
import frc.robot.commands.SlowMode;
import frc.robot.commands.LockToDirection;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.utils.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import javax.xml.validation.Validator;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DrivetrainSubsystem m_robotDrive = DrivetrainSubsystem.getInstance();
  private final Limelight ll = Limelight.getInstance();

  // The driver's controller
  PS5Controller m_driverController = new PS5Controller(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  // AUTOS
  private SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {

    // AUTO CHOOSER
    SmartDashboard.putData("Auto Chooser", this.chooser);
    chooser.addOption("Simple", new PathPlannerAuto("Simple"));
    chooser.addOption("1", new PathPlannerAuto("Wing-1 (Left Amp) to Center Left Auto"));
    chooser.setDefaultOption("Simple", new PathPlannerAuto("Simple"));
    
    NamedCommands.registerCommand("Lock", new RunCommand(() -> m_robotDrive.setX()));

    // Configure the button bindings
    configureButtonBindings();
  
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
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
    
    //Aim while moving
    new JoystickButton(m_driverController, Button.kR1.value)
                      .whileTrue(new RunCommand(() -> m_robotDrive.aimToTarget(
                        MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)
                        )));

    //Slow mode while moving
    new JoystickButton(m_driverController, Button.kL1.value).whileTrue(
                      new RunCommand(() -> m_robotDrive.slowMode(
                        MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband))));
                        
    //Cardinal positions
    new JoystickButton(m_driverController, Button.kCircle.value).whileTrue(new RunCommand(() -> lockToCardinal(100)));
    new JoystickButton(m_driverController, Button.kTriangle.value).whileTrue(new RunCommand(() -> lockToCardinal(0)));
    new JoystickButton(m_driverController, Button.kSquare.value).whileTrue(new RunCommand(() -> lockToCardinal(200)));
    new JoystickButton(m_driverController, Button.kCross.value).whileTrue(new LockToDirection(
                                                                          MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                                                          MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                                                     0));

  }

  private void lockToCardinal(double goal){
    m_robotDrive.cardinalDirection(
                        MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
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


}   