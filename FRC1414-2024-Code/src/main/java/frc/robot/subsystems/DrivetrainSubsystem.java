// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class DrivetrainSubsystem extends SubsystemBase {

  private static DrivetrainSubsystem instance;
  private VisionSubsystem visionSubsystem;
  private Field2d field;
  private PIDController rotController;
  private PIDController translateXController;
  private PIDController translateYController;
  private final SwerveDrivePoseEstimator poseEstimator;
  private double poseAngle;
  private double vectorDistance;

  private final AHRS m_gyro = new AHRS();
  private int[] cardinalAngles;
  
  // Rotation of chassis
  private double m_currentRotation;

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

    //PID Controller Initialization
    rotController = new PIDController(0.01, 0, 0);
    translateXController = new PIDController(1, 0, 0);
    translateYController = new PIDController(1, 0, 0);
    rotController.enableContinuousInput(-180, 180);

    poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, 
                            Rotation2d.fromDegrees(-m_gyro.getAngle()), 
                            getSwerveModulePositions(), 
                            new Pose2d(),
                            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5)), // Tune | State estimation deviation
                            VecBuilder.fill(0.75, 0.75, Units.degreesToRadians(130))); // Tune | Vision estimation deviation

    visionSubsystem = VisionSubsystem.getInstance();
    field = new Field2d();
    m_currentRotation = 0.0;
    vectorDistance = 0;
    poseAngle = 0;
    cardinalAngles = new int[]{0, 90, -90};
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

  //Robot relative drive for path planner
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
      var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
      setModuleStates(states);
  }

  //Robot field relative drive for path planner
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){
      ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation());
      driveRobotRelative(robotRelative);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Distance from Tag", distanceFromTarget());
    SmartDashboard.putNumber("Height of Tag", visionSubsystem.getFrontCamera().getHeightFromID());
    SmartDashboard.putNumber("X Est Pose", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Y Est Pose", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Est Rot", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putData("Field", field);
    poseToTagDistance(11);
    SmartDashboard.putNumber("Vector Distance", vectorDistance);
    SmartDashboard.putBoolean("Target is Appropiate", visionSubsystem.getFrontCamera().targetAppropiate(7));

    //Update odomotry
    poseEstimator.update(Rotation2d.fromDegrees(-m_gyro.getAngle()), getSwerveModulePositions());

    var visionEst = visionSubsystem.getEstimatedGlobalPose(visionSubsystem.getPoseEstimator(), visionSubsystem.getFrontCamera());
    visionEst.ifPresent(
        est -> {
          poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        }

    );
    field.setRobotPose(getPose());
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
  
  public void aimToTarget(double xSpeed, double ySpeed, int id){

    if(visionSubsystem.getFrontCamera().targetDetected() && visionSubsystem.getFrontCamera().targetAppropiate(id)){
      double yaw = visionSubsystem.getFrontCamera().getYaw();
      if(!(yaw < 0 && yaw > -DriveConstants.kYawThreshold || yaw > 0 && yaw < DriveConstants.kYawThreshold)){
        drive(xSpeed, ySpeed, new ProfiledPIDController(0.015, 0, 0, new TrapezoidProfile.Constraints(1, 1))
                                              .calculate(yaw, 
                                              0), 
                                              true);
      }
    }
    else {
      rotateToPose(xSpeed, ySpeed, id);
    }
  }

  //Drive to a specific pose from current pose
  public void driveToPose(Pose2d target){

    translateXController.setSetpoint(target.getX());
    translateYController.setSetpoint(target.getY());
    rotController.setSetpoint(target.getRotation().getDegrees());
    
    double rotationVal = rotController.calculate(-(MathUtil.inputModulus(m_gyro.getYaw(), -180, 180)), rotController.getSetpoint());
    double translateX = translateXController.calculate(poseEstimator.getEstimatedPosition().getX(), translateXController.getSetpoint());
    double translateY = translateYController.calculate(poseEstimator.getEstimatedPosition().getY(), translateYController.getSetpoint());
    drive(translateX, translateY, rotationVal, true);
  }
  
  public double poseToTagDistance(int id){
    Vector<N2> robotVector = translationToVector(getPose().getTranslation());
    Vector<N2> goalVector = translationToVector(FieldConstants.getTagTranslation(id));

    //Vector from goal to robot
    goalVector = goalVector.minus(robotVector);
    vectorDistance = goalVector.norm();
    return vectorDistance;
  }

  public void rotateToPose(double xSpeed, double ySpeed, int id){
    Translation2d target = FieldConstants.getTagTranslation(id);
    double robotToTargetX = getPose().getX() - target.getX();   
    double robotToTargetY = getPose().getY() - target.getY();
    double additive = 0;
    if(robotToTargetX < 0) { additive = 180; } else { additive = 0; }
    poseAngle = Math.toDegrees(Math.atan(robotToTargetY / robotToTargetX)) + additive;
    
    rotController.setSetpoint(poseAngle);
    double rotationVal = rotController.calculate(-(MathUtil.inputModulus(m_gyro.getYaw(), -180, 180)), rotController.getSetpoint());
    if(poseToTagDistance(id) < FieldConstants.kDistanceThreshold) {
      if(id == FieldConstants.kRedSpeakerID || id == FieldConstants.kBlueSpeakerID){
        cardinalDirection(xSpeed, ySpeed, cardinalAngles[0]);
      }
      else if(FieldConstants.isRedTag(id)){
        cardinalDirection(xSpeed, ySpeed, cardinalAngles[1]);
      }
      else {
        cardinalDirection(xSpeed, ySpeed, cardinalAngles[2]);
      }
    }
    else{ drive(xSpeed, ySpeed, rotationVal, true);}
  }
  
  //Lock the robot to field-oriented N-E-S-W
  public void cardinalDirection(double xSpeed, double ySpeed, double goal){
    rotController.setSetpoint(goal);
    double rotationVal = rotController.calculate(-(MathUtil.inputModulus(m_gyro.getYaw(), -180, 180)), rotController.getSetpoint());
    drive(xSpeed, ySpeed, rotationVal, true);

  }
  
  //Move the robot with precision
  public void slowMode(double xSpeed, double ySpeed, double rot){
    drive(xSpeed * DriveConstants.kSlowMode, 
          ySpeed * DriveConstants.kSlowMode, 
          rot * DriveConstants.kSlowMode, 
          true);
  }

  public Vector<N2> translationToVector(Translation2d translation){
    return VecBuilder.fill(translation.getX(), translation.getY());
  }

}