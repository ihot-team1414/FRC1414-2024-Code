package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Drive;
import frc.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    private double lastTX;
    private double lastDistance;
    private double lastMeasurementTime;
    private boolean isStale;

    public VisionSubsystem() {
        this.lastTX = 0;
        this.lastDistance = 0;
        this.lastMeasurementTime = 0;
        this.isStale = true;
    }

    public static synchronized VisionSubsystem getInstance() {
        if (instance == null) {
            instance = new VisionSubsystem();
        }
        return instance;
    }

    public Optional<Double> getTX() {
        // // removed because drivetrain doesn't like turning to the ocrrect angle

        // if (VisionConstants.kShootOnTheMove) {
        // ChassisSpeeds speeds =
        // DrivetrainSubsystem.getInstance().getRobotRelativeSpeeds();
        // double cosAngle = DrivetrainSubsystem.getInstance().getHeading().getCos();
        // if (speeds.omegaRadiansPerSecond < 1) {
        // return isStale ? Optional.empty()
        // : Optional.of(lastTX
        // + (-speeds.vyMetersPerSecond * cosAngle * VisionConstants.kAngleConverter
        // / VisionConstants.kEstimatedShotSpeed));
        // }
        // }
        return isStale ? Optional.empty() : Optional.of(lastTX);
    }

    public Optional<Double> getDistance() {

        if (VisionConstants.kShootOnTheMove) {
            ChassisSpeeds speeds = DrivetrainSubsystem.getInstance().getRobotRelativeSpeeds();
            if (speeds.omegaRadiansPerSecond < 1) {
                return isStale ? Optional.empty()
                        : Optional.of(lastDistance
                                + ((lastDistance / VisionConstants.kEstimatedShotSpeed) * speeds.vxMetersPerSecond));
            }
        }

        return isStale ? Optional.empty() : Optional.of(lastDistance);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Distance", lastDistance);
        SmartDashboard.putNumber("TX", lastTX % 360);
        SmartDashboard.putBoolean("Vision stale?", isStale);

        if (LimelightHelpers.getTV("limelight-front")) {
            lastTX = DrivetrainSubsystem.getInstance().getHeading().getDegrees()
                    - (LimelightHelpers.getTX("limelight-front"));
            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front");
            lastDistance = tagPose.getTranslation().getNorm();
            isStale = false;

            lastMeasurementTime = Timer.getFPGATimestamp();
        } else if (LimelightHelpers.getTV("limelight-left")) {
            lastTX = -LimelightHelpers.getTX("limelight-left") + VisionConstants.kLeftLimelightOffset
                    + DrivetrainSubsystem.getInstance().getHeading().getDegrees();
            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-left");
            lastDistance = tagPose.getTranslation().getNorm();
            isStale = false;

            lastMeasurementTime = Timer.getFPGATimestamp();
        } else if (LimelightHelpers.getTV("limelight-right")) {
            lastTX = -LimelightHelpers.getTX("limelight-right") + VisionConstants.kRightLimelightOffset
                    + DrivetrainSubsystem.getInstance().getHeading().getDegrees();
            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-right");
            lastDistance = tagPose.getTranslation().getNorm();
            isStale = false;

            lastMeasurementTime = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - lastMeasurementTime > VisionConstants.kCacheTimeout) {
            isStale = true;
        }
    }

}