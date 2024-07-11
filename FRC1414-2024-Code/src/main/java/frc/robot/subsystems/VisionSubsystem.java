package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    private double lastTX;
    private double lastDistance;
    private double lastMeasurementTime;
    private boolean isStale;

    public VisionSubsystem() {
        this.lastTX = 0;  //TX represents the horixontal angle between the robot's current heading and the target detected by the Limelight camera.
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
        if (VisionConstants.kShootOnTheMove) {
            ChassisSpeeds speeds = DrivetrainSubsystem.getInstance().getRobotRelativeSpeeds();
            if (speeds.omegaRadiansPerSecond < 1) {
                return isStale ? Optional.empty()
                        : Optional.of(lastDistance
                                + ((lastDistance / VisionConstants.kEstimatedShotSpeed) * speeds.vxMetersPerSecond));
            }
        }

        return isStale ? Optional.empty() : Optional.of(lastDistance); //Optional is built into Java and is used to draw attention to null pointers so devs prevent them
        //using the Optional.empty() value will result in nullpointerexception
        //Optional.of(lastDistance) is essentially the same as just using lastDistance
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight Distance", lastDistance);
        SmartDashboard.putNumber("Limelight Angular Error", lastTX % 360);

        if (LimelightHelpers.getTV("limelight-front")) {
            lastTX = DrivetrainSubsystem.getInstance().getHeading().getDegrees()
                    - (LimelightHelpers.getTX("limelight-front"));
            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front");
            lastDistance = tagPose.getTranslation().getNorm();
            isStale = false;

            lastMeasurementTime = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - lastMeasurementTime > VisionConstants.kCacheTimeout) {
            isStale = true;
        }
    }

}
