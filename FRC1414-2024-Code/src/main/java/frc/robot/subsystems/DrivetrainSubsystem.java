// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class DrivetrainSubsystem extends SubsystemBase {

  private static DrivetrainSubsystem instance;
  private VisionSubsystem visionSubsystem;
  private Field2d field;
  private PIDController rotController;
  private final SwerveDrivePoseEstimator poseEstimator;
  private double x = 0;
  
  // The gyro sensor
  private final AHRS m_gyro = new AHRS();
  
  // Rotation of chassis
  private double m_currentRotation = 0.0;

  public static synchronized DrivetrainSubsystem getInstance() {
    if (instance == null) {
      instance = new DrivetrainSubsystem();
    }
    return instance;
  }

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);


/*
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  */

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
      
      AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s fucking useless
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    rotController = new PIDController(0.01, 0, 0);
    rotController.enableContinuousInput(-180, 180);

    poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, 
                            Rotation2d.fromDegrees(-m_gyro.getAngle()), 
                            getSwerveModulePositions(), 
                            new Pose2d(), /* vs getPose()? */
                            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5)), // Tune | State estimation deviation
                            VecBuilder.fill(0.75, 0.75, Units.degreesToRadians(130))); // Tune | Vision estimation deviation

    visionSubsystem = VisionSubsystem.getInstance();
    field = new Field2d();

  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    return(new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    });
  }

  private SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
      var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
      setModuleStates(states);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){
      ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation());
      driveRobotRelative(robotRelative);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Distance", distanceFromTarget());
    SmartDashboard.putNumber("Height", visionSubsystem.getFrontCamera().getHeightFromID());
    SmartDashboard.putNumber("x pose", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("y pose", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("x rot", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putData("Field", field);

    poseEstimator.update(Rotation2d.fromDegrees(-m_gyro.getAngle()), getSwerveModulePositions());

    var visionEst = visionSubsystem.getEstimatedGlobalPose(/*visionSubsystem.getPoseEstimator(), visionSubsystem.getFrontCamera()*/);
    visionEst.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = visionSubsystem.getEstimationStdDevs(estPose);
          x = est.estimatedPose.toPose2d().getX();
          poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        }

    );

    field.setRobotPose(getPose());

    /*
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });*/
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        getSwerveModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeedCommanded = xSpeed;
    ySpeedCommanded = ySpeed;
    m_currentRotation = rot;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(Translation2d translation, double rot){
    drive(translation.getX(), translation.getY(), rot, true);
  }

  /**zero
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public double distanceFromTarget(){
    return visionSubsystem.getFrontCamera().getDistance();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  public double getYaw(){
    return m_gyro.getYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public SwerveDrivePoseEstimator getPoseEstimator(){
    return poseEstimator;
  }
  
  public void aimToTarget(double xSpeed, double ySpeed){

    if(visionSubsystem.getFrontCamera().targetDetected() && visionSubsystem.getFrontCamera().targetAppropiate()){
      double yaw = visionSubsystem.getFrontCamera().getYaw();
      if(!(yaw < 0 && yaw > -DriveConstants.kYawThreshold || yaw > 0 && yaw < DriveConstants.kYawThreshold)){
        drive(xSpeed, ySpeed, new ProfiledPIDController(0.015, 0, 0, new TrapezoidProfile.Constraints(1, 1))
                                              .calculate(yaw, 
                                              0), 
                                              true);
      }
    }
    // else { drive(xSpeed, ySpeed, rot, true); } add robot move during lock?
  }

  
  public void cardinalDirection(double xSpeed, double ySpeed, double goal){
    
    rotController.setSetpoint(goal);
    double rotationVal = rotController.calculate(-(MathUtil.inputModulus(m_gyro.getYaw(), -180, 180)), rotController.getSetpoint());
    drive(xSpeed, ySpeed, rotationVal, true);

  }
  
  public void slowMode(double xSpeed, double ySpeed, double rot){
    drive(xSpeed * DriveConstants.kSlowMode, 
          ySpeed * DriveConstants.kSlowMode, 
          rot * DriveConstants.kSlowMode, 
          true);
  }

}