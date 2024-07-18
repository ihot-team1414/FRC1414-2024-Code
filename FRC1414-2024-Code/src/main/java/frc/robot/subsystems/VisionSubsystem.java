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
            if (speeds.omegaRadiansPerSecond < 1) { //ensures calculation only proceeds if the robot is not rotating quickly, as high rotational speeds could affect accuracy.
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
        SmartDashboard.putNumber("Limelight Distance", lastDistance); //sends the lastDistance value to the SmartDashboard
        SmartDashboard.putNumber("Limelight Angular Error", lastTX % 360);

        if (LimelightHelpers.getTV("limelight-front")) { //getTV stands for "get Target Validity" or similar, indicating whether the Limelight has detected a valid target.
            lastTX = DrivetrainSubsystem.getInstance().getHeading().getDegrees() - (LimelightHelpers.getTX("limelight-front"));
                    /* transforms the TX value from a relative angle (relative to the robot's heading) into a global angle (relative to the field).
                    B/c DrivetrainSubsystem.getInstance().getHeading().getDegrees() gives current global heading. (ask AI for ex if confused) */
            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front");
            lastDistance = tagPose.getTranslation().getNorm(); //vector math to get last distance
            isStale = false; //resets stale

            lastMeasurementTime = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - lastMeasurementTime > VisionConstants.kCacheTimeout) {
            isStale = true;
        }
    }

}
