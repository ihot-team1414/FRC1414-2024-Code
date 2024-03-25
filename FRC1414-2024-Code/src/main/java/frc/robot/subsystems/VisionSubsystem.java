package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
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
        return isStale ? Optional.empty() : Optional.of(lastTX);
    }

    public Optional<Double> getDistance() {
        return isStale ? Optional.empty() : Optional.of(lastDistance);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Distance", lastDistance);
        SmartDashboard.putNumber("TX", lastTX);
        SmartDashboard.putBoolean("Vision stale?", isStale);

        if (LimelightHelpers.getTV("limelight-front")) {
            lastTX = LimelightHelpers.getTX("limelight-front");
            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front");
            lastDistance = tagPose.getTranslation().getNorm();
            isStale = false;

            lastMeasurementTime = Timer.getFPGATimestamp();
        } else if (LimelightHelpers.getTV("limelight-left")) {
            lastTX = LimelightHelpers.getTX("limelight-left") + VisionConstants.kLeftLimelightOffset;
            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-left");
            lastDistance = tagPose.getTranslation().getNorm();
            isStale = false;

            lastMeasurementTime = Timer.getFPGATimestamp();
        } else if (LimelightHelpers.getTV("limelight-left")) {
            lastTX = LimelightHelpers.getTX("limelight-left") + VisionConstants.kRightLimelightOffset;
            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-left");
            lastDistance = tagPose.getTranslation().getNorm();
            isStale = false;

            lastMeasurementTime = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - lastMeasurementTime > VisionConstants.kCacheTimeout) {
            isStale = true;
        }
    }

}