package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    
    private String networkTable;
    private PhotonCamera camera;
    private PhotonPipelineResult result;

    public PhotonVision(String networkTable){
        this.networkTable = networkTable;
        camera = new PhotonCamera(networkTable);
    }


    public boolean targetDetected(){
        PhotonPipelineResult result = camera.getLatestResult();
        // Check if the latest result has any targets.
        return(result.hasTargets());
    }

    public PhotonTrackedTarget getBestTarget(){
        
        if(targetDetected()){
            return(result.getBestTarget());
        }
        return null;
    }

    @Override
    public void periodic(){
        result = camera.getLatestResult();
    }

}
