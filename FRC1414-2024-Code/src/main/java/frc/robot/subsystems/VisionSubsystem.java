package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;


public class VisionSubsystem extends SubsystemBase{
    
    private static VisionSubsystem instance;
    private AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private double previous = 0;

    PhotonVisionHelper frontCamera = new PhotonVisionHelper("frontCamera");    
    PhotonVisionHelper backCamera = new PhotonVisionHelper("backCamera");
    PhotonPoseEstimator visionEstimatorFront;

    private VisionSubsystem(){
      visionEstimatorFront = new PhotonPoseEstimator(fieldLayout, 
                                                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                    frontCamera.getCamera(),
                                                    VisionConstants.kFrontCameraToRobot);

      visionEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
                  
    @Override
    public void periodic(){
      SmartDashboard.putBoolean("rgn", getEstimatedGlobalPose().isPresent());
      SmartDashboard.putBoolean("detect", frontCamera.targetDetected());
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(/*PhotonPoseEstimator photonPose, PhotonVisionHelper camera*/) {
      var visionEst = visionEstimatorFront.update();
      double latest = frontCamera.getCamera().getLatestResult().getTimestampSeconds();
      boolean newResult = Math.abs(latest - previous) > 1e-5;
      if (newResult) previous = latest;
      return visionEst;
    }

    public PhotonPoseEstimator getVisionPoseEstimatorFront(){
      return visionEstimatorFront;
    }

    //Get front camera
    public PhotonVisionHelper getFrontCamera(){
        return frontCamera;
    }

    //Get back camera
    public PhotonVisionHelper getBackCamera(){
        return backCamera;
    }

    //Get pose estimator
    public PhotonPoseEstimator getPoseEstimator(){
        return visionEstimatorFront;
    }

    public static VisionSubsystem getInstance() {
        if (instance == null) {
          instance = new VisionSubsystem();
        }
    
        return instance;
      }

}
