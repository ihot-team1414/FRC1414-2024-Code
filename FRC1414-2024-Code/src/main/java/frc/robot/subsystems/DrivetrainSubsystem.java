package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.utils.LimelightHelpers;

public class DrivetrainSubsystem extends SubsystemBase {
    private static DrivetrainSubsystem instance;

    /*
     * Initialize swerve modules.
     */
    private final MAXSwerveModule frontLeftSwerve = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule frontRightSwerve = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule rearLeftSwerve = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule rearRightSwerve = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    /*
     * Initialize gryo.
     */
    private final Pigeon2 pigeon = new Pigeon2(DriveConstants.kPigeonCanID);

    /*
     * Initialize odometry.
     * TODO: Tune odometry standard deviations.
     */
    SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            getHeading(), new SwerveModulePosition[] {
                    frontLeftSwerve.getPosition(),
                    frontRightSwerve.getPosition(),
                    rearLeftSwerve.getPosition(),
                    rearRightSwerve.getPosition()
            }, new Pose2d(),
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5)),
            VecBuilder.fill(0.75, 0.75, 99999999));

    /*
     * Initialize visualization.
     */
    private Field2d field = new Field2d();

    public DrivetrainSubsystem() {

        AutoBuilder.configureHolonomic(
                this::getCurrentPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(3, 0, 0),
                        new PIDConstants(3, 0, 0),
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        0.42,
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get().equals(DriverStation.Alliance.Red);
                    }
                    return false;
                },
                this);

    };

    public static synchronized DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }
        return instance;
    }

    // Robot relative drive for path planner
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, AutoConstants.kMaxSpeedMetersPerSecond);
        setModuleStates(states);
    }

    // Robot field relative drive for path planner
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds,
                getHeading());
        driveRobotRelative(robotRelative);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    private SwerveModuleState[] getModuleStates() {
        return (new SwerveModuleState[] {
                frontLeftSwerve.getState(),
                frontRightSwerve.getState(),
                rearLeftSwerve.getState(),
                rearRightSwerve.getState()
        });
    }

    public void drive(Transform2d transform, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(transform.getX(), transform.getY(),
                                transform.getRotation().getRadians(),
                                getHeading())
                        : new ChassisSpeeds(transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        setModuleStates(swerveModuleStates);
    }

    public void lock() {
        SwerveModuleState[] lockStates = {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };
        setModuleStates(lockStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeftSwerve.setDesiredState(desiredStates[0]);
        frontRightSwerve.setDesiredState(desiredStates[1]);
        rearLeftSwerve.setDesiredState(desiredStates[2]);
        rearRightSwerve.setDesiredState(desiredStates[3]);
    }

    /*
     * Sensor Methods
     */
    public Pose2d getCurrentPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                getHeading(),
                getSwerveModulePositions(),
                pose);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftSwerve.getPosition(),
                frontRightSwerve.getPosition(),
                rearLeftSwerve.getPosition(),
                rearRightSwerve.getPosition()
        };
    }

    public void resetEncoders() {
        frontLeftSwerve.resetEncoders();
        rearLeftSwerve.resetEncoders();
        frontRightSwerve.resetEncoders();
        rearRightSwerve.resetEncoders();
    }

    /*
     * Returns current drivetrain heading in the WPILIB coordinate system.
     */
    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    public double getDegrees(){
        return pigeon.getAngle();
    }

    //TODO: Fix gyro reset to properly relocate the robot
    public void resetHeading() {
        pigeon.setYaw(0);
        odometry.resetPosition(getHeading(), getSwerveModulePositions(), getCurrentPose());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro", getHeading().getDegrees() % 360);
        SmartDashboard.putData("Field", field);

        odometry.update(getHeading(), getSwerveModulePositions());
        addVisionMeasurement("limelight-front");
        addVisionMeasurement("limelight-left");
        addVisionMeasurement("limelight-right");

        field.setRobotPose(odometry.getEstimatedPosition());
    }

    public void addVisionMeasurement(String limelight) {
        LimelightHelpers.SetRobotOrientation(limelight, odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        if (LimelightHelpers.getTV(limelight)) {
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

            if(!(Math.abs(pigeon.getRate()) > 720) && !(mt2.tagCount == 0)) {
                odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                odometry.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }
}
