// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PivotConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //PIVOT MOTORS
  private TalonFX pivotMotor1 = new TalonFX(30);
  private TalonFX pivotMotor2 = new TalonFX(31);

  //SHOOTER MOTORS
  private TalonFX shooterMotor1 = new TalonFX(32);
  private TalonFX shooterMotor2 = new TalonFX(33);

  //INTAKE MOTORS
  private TalonFX intake1 = new TalonFX(34);
  private TalonFX intake2 = new TalonFX(35);

  //FOLLOWER CONTROLS (S - Shooter, P - Pivot, I - Intake)
  private Follower followerS = new Follower(shooterMotor1.getDeviceID(), true);
  private Follower followerP = new Follower(pivotMotor1.getDeviceID(), true);
  private Follower followerI = new Follower(intake1.getDeviceID(), true);

  //CONTROL OUTPUT (SET ENABLEFOC TO TRUE/FALSE)
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0, true, false, false, false);
  private PositionDutyCycle positionMove = new PositionDutyCycle(0, 0, true, 0, 0, false, false, false);

  //INTAKE SENSOR
  private TimeOfFlight intakeSensor = new TimeOfFlight(50);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
    m_robotContainer = new RobotContainer();
    
    /*
    //PIVOT NEUTRAL MODE
    pivotMotor1.setNeutralMode(NeutralModeValue.Brake);

    //INTAKE NEUTRAL MODE
    intake1.setNeutralMode(NeutralModeValue.Brake);

    //SHOOTER NEUTRAL MODE
    shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
    
    //PIVOT PID CONTROLS
    TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
    Slot0Configs configs = new Slot0Configs();
    
    configs.kP = 0.05;
    configs.kI = 0;
    configs.kD = 0.0;

    pivotMotorConfig.withSlot0(configs);
    pivotMotor1.getConfigurator().apply(pivotMotorConfig);
    
    //SET PIVOT AND INTAKE TO FOLLOW 
    shooterMotor2.setControl(followerS);
    pivotMotor2.setControl(followerP);
    intake2.setControl(followerI);

    //CHANGE SAMPLE TIME FOR QUICKER RESPONSE --- CHECK ROBOT PERIODIC FOR FUNCTION
    intakeSensor.setRangingMode(RangingMode.Short, 24);

    //R1 TO RUN SHOOTERS (AT DIFFERENTIAL)
    new JoystickButton(new PS5Controller(0), PS5Controller.Button.kR1.value).whileTrue(new RunCommand(() -> shooterMotor1.setControl(dutyCycleOut.withOutput(1))));
    //new JoystickButton(new PS5Controller(0), PS5Controller.Button.kR1.value).whileTrue(new RunCommand(() -> shooterMotor2.setControl(dutyCycleOut.withOutput(-0.95))));

    //L1 TO STOP SHOOTERS
    new JoystickButton(new PS5Controller(0), PS5Controller.Button.kL1.value).whileTrue(new RunCommand(() -> shooterMotor1.set(0)));
    //new JoystickButton(new PS5Controller(0), PS5Controller.Button.kL1.value).whileTrue(new RunCommand(() -> shooterMotor2.set(0)));

    //TRIANGLE TO INTAKE
    new JoystickButton(new PS5Controller(0), PS5Controller.Button.kTriangle.value).whileTrue(new ParallelCommandGroup(new RunCommand(() -> intake1.set(-1))));//, new RunCommand(() -> shooterMotor1.set(0.1))));
    
    //SQUARE TO STOP INTAKE
    new JoystickButton(new PS5Controller(0), PS5Controller.Button.kSquare.value).whileTrue(new ParallelCommandGroup(new RunCommand(() -> intake1.set(0))));//, new RunCommand(() -> shooterMotor1.set(0))));

    //CIRCLE TO OUTTAKE
    new JoystickButton(new PS5Controller(0), PS5Controller.Button.kCircle.value).whileTrue(new RunCommand(() -> intake1.set(0.75)));
       
    //R2 TO RAISE PIVOT
    new JoystickButton(new PS5Controller(0), PS5Controller.Button.kR2.value).whileTrue(new RunCommand(() -> pivotMotor1.setControl(positionMove.withPosition(7.25))));
    
    //L2 TO LOWER PIVOT
    new JoystickButton(new PS5Controller(0), PS5Controller.Button.kL2.value).whileTrue(new RunCommand(() -> pivotMotor1.setControl(positionMove.withPosition(1))));
    */
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    TalonFX pivot1 = new TalonFX(30);
    TalonFX pivot2 = new TalonFX(31);

    pivot1.setNeutralMode(NeutralModeValue.Brake);
    pivot2.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  

  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //shooterMotor2.setControl(follower);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    /*
    SmartDashboard.putNumber("Speed", shooterMotor1.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Position", pivotMotor1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Position1", pivotMotor2.getPosition().getValueAsDouble());

    //GET RANGE IS IN MILLIMETERS
    
    if(intakeSensor.getRange() <= 228){
      Commands.waitSeconds(10);
      intake1.stopMotor();
    } */
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {} 

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
