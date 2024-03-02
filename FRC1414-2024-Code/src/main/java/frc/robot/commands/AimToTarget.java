// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionHelper;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AimToTarget extends Command {
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final PhotonVisionHelper frontCamera = VisionSubsystem.getInstance().getFrontCamera();
  private double xSpeed;
  private double ySpeed;
  double yaw = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public AimToTarget(double xSpeed, double ySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    //Add PhotonVisionHelper as a requirement?
    addRequirements(drivetrain);
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(frontCamera.targetDetected() && frontCamera.targetAppropiate()){
      yaw = frontCamera.getYaw();
      if(!(yaw < 0 && yaw > -DriveConstants.kYawThreshold || yaw > 0 && yaw < DriveConstants.kYawThreshold)){
        drivetrain.drive(xSpeed, ySpeed, new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(0.05, 0.05))
                                              .calculate(yaw, 
                                              0), 
                                              true);
      }
    }
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
