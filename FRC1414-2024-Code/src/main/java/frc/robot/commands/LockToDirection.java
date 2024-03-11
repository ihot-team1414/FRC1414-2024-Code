// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionHelper;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LockToDirection extends Command {
  
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private double xSpeed;
  private double ySpeed;
  private double goal;
  private PIDController rotController = new PIDController(0.01, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public LockToDirection(double xSpeed, double ySpeed, double goal) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    //Add PhotonVisionHelper as a requirement?
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.goal = goal;
      
    rotController.enableContinuousInput(-180, 180);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotController.setSetpoint(goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotationVal = rotController.calculate(-(MathUtil.inputModulus(drivetrain.getYaw(), -180, 180)), rotController.getSetpoint());
    drivetrain.drive(xSpeed, ySpeed, rotationVal, true);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
