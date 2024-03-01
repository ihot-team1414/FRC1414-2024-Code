package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionHelper extends SubsystemBase {
    
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public PhotonVisionHelper(String networkTable){
        camera = new PhotonCamera(networkTable);
    }

    
    public double getDistance(){
        return Math.abs(PhotonUtils.calculateDistanceToTargetMeters(
                        VisionConstants.kFrontCameraHeight, 
                        getHeightFromID(), 
                        VisionConstants.kFrontCameraPitch, 
                        Math.toRadians(getPitch())));
    }

    public double getHeightFromID(){
        
        if(targetDetected()){
        
            int id = result.getBestTarget().getFiducialId();
            if(id >= 11) { return VisionConstants.kStageHeight; }
            else if (id == 3 || id == 4 || id == 7 || id == 8) { return VisionConstants.kSpeakerHeight; }
            else { return VisionConstants.kAmpHeight; } 
        }
        return 0.0;
    
    }

    public boolean targetDetected(){
        result = camera.getLatestResult();
        return(result.hasTargets());
    }

    public double getYaw(){
        if(targetDetected()){
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    public double getPitch(){
        if(targetDetected()){ 
            return result.getBestTarget().getPitch();
        }
        return 0.0;
    }

    
    @Override
    public void periodic(){

    }

}
