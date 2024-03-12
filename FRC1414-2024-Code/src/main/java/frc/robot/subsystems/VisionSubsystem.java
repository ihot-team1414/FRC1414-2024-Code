package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;


public class VisionSubsystem extends SubsystemBase{
    
    private static VisionSubsystem instance;
    private AprilTagFieldLayout fieldLayout;
    private Optional<Alliance> allianceColor;
    private double previous;

    PhotonVisionHelper frontCamera = new PhotonVisionHelper("frontCamera");    
    PhotonVisionHelper backCamera = new PhotonVisionHelper("backCamera");
    PhotonPoseEstimator visionEstimatorFront;

    private VisionSubsystem(){

      fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      allianceColor = DriverStation.getAlliance();
      previous = 0;
      
      if(allianceColor.isPresent()){ 
        if(allianceColor.get().equals(Alliance.Red)){ fieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide); }
        else{ fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide); }}


      visionEstimatorFront = new PhotonPoseEstimator(fieldLayout, 
                                                    PoseStrategy.LOWEST_AMBIGUITY,
                                                    frontCamera.getCamera(),
                                                    VisionConstants.kFrontCameraToRobot);

      visionEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
                  
    @Override
    public void periodic(){
      SmartDashboard.putBoolean("Updating Pose?", getEstimatedGlobalPose(visionEstimatorFront, frontCamera).isPresent());
      SmartDashboard.putBoolean("Target Detected", frontCamera.targetDetected());
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator photonPose, PhotonVisionHelper camera) {
      var visionEst = photonPose.update();
      double latest = camera.getCamera().getLatestResult().getTimestampSeconds();
      boolean newResult = Math.abs(latest - previous) > 1e-5;
      if (newResult) previous = latest;
      return visionEst;
    }

    // Get the standard deviations of the estimated pose
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(30));
        var targets = frontCamera.getCamera().getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = visionEstimatorFront.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(30));
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
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
