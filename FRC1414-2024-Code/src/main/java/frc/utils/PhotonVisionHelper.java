package frc.utils;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class PhotonVisionHelper {
    
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private Optional<Alliance> allianceColor;
    private double[] configs;

    public PhotonVisionHelper(String networkTable, double[] configs){
        camera = new PhotonCamera(networkTable);
        allianceColor = DriverStation.getAlliance();
        this.configs = configs;
    }

    public double getDistance(){
        return Math.abs(PhotonUtils.calculateDistanceToTargetMeters(
                        configs[0], //Camera height
                        getHeightFromID(), 
                        configs[1], //Camera pitch
                        Math.toRadians(getPitch())));
    }

    public double getHeightFromID(){
        if(targetDetected()){
            int id = result.getBestTarget().getFiducialId();
            if(id >= 11) { return FieldConstants.kStageHeight; }
            else if (id == 3 || id == 4 || id == 7 || id == 8) { return FieldConstants.kSpeakerHeight; }
            else { return FieldConstants.kAmpHeight; } 
        }
        return 0.0;
    }

    public boolean targetAppropiate(int id){
        if(targetDetected() && allianceColor.isPresent()){
            int tag = result.getBestTarget().getFiducialId();
             if(allianceColor.get().equals(Alliance.Red) && tag == (id) || 
                allianceColor.get().equals(Alliance.Blue) && tag == (id)){
                return true;
            }
        }
        return false;
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

    public PhotonCamera getCamera(){
        return camera;
    }

}
