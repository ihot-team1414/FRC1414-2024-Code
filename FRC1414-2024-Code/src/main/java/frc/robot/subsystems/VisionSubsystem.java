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
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private AprilTagFieldLayout fieldLayout;
    private Optional<Alliance> allianceColor;
    private double previous = 0;

    PhotonVisionHelper frontCamera = new PhotonVisionHelper("frontCamera");    
    PhotonVisionHelper backCamera = new PhotonVisionHelper("backCamera");
    PhotonPoseEstimator visionEstimatorFront;

    private VisionSubsystem(){

      fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      allianceColor = DriverStation.getAlliance();
      
      if(allianceColor.isPresent()){ 
        if(allianceColor.get().equals(Alliance.Red)){ fieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide); }
        else{ fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide); }}


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
      SmartDashboard.putNumber("previous", previous);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(/*PhotonPoseEstimator photonPose, PhotonVisionHelper camera*/) {
      var visionEst = visionEstimatorFront.update();
      double latest = frontCamera.getCamera().getLatestResult().getTimestampSeconds();
      boolean newResult = Math.abs(latest - previous) > 1e-5;
      if (newResult) previous = latest;
      return visionEst;
    }

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
