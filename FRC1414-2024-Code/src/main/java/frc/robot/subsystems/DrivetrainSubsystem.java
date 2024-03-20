package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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
    private final AHRS gyro = new AHRS();

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
            VecBuilder.fill(0.75, 0.75, Units.degreesToRadians(130)));

    /*
     * Initialize visualization.
     */
    private Field2d field = new Field2d();

    public DrivetrainSubsystem() {
        SmartDashboard.putData(field);
    }

    public static synchronized DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }
        return instance;
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
                new SwerveModulePosition[] {
                        frontLeftSwerve.getPosition(),
                        frontRightSwerve.getPosition(),
                        rearLeftSwerve.getPosition(),
                        rearRightSwerve.getPosition()
                },
                pose);
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
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public void resetHeading() {
        gyro.zeroYaw();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro", getHeading().getDegrees());
        field.setRobotPose(odometry.getEstimatedPosition());
    }
}