package frc.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.VisionConstants;

public class VisionPoseEstimation {
    
    private static VisionPoseEstimation instance;
    // Change this to match the name of your camera
    PhotonCamera at1 = new PhotonCamera("at1");
    PhotonCamera at2 = new PhotonCamera("at2");
    PhotonPipelineResult result;

    private final AHRS gyro = new AHRS();

    public boolean targetDetected(){
        PhotonPipelineResult result = at1.getLatestResult();
        // Check if the latest result has any targets.
        return(result.hasTargets());
    }

    public PhotonTrackedTarget getBestTarget(){
        
        if(targetDetected()){
            return(result.getBestTarget());
        }

        return null;
    }

    public static VisionPoseEstimation getInstance() {
        if (instance == null) {
          instance = new VisionPoseEstimation();
        }
    
        return instance;
      }

}
