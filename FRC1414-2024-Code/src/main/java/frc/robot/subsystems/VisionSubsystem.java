package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase{
    
    private static VisionSubsystem instance;
    private static DrivetrainSubsystem driveTrain;

    PhotonVisionHelper frontCamera = new PhotonVisionHelper("frontCamera");
    PhotonVisionHelper backCamera = new PhotonVisionHelper("backCamera");

    //Get front camera
    public PhotonVisionHelper getFrontCamera(){
        return frontCamera;
    }

    //Get back camera
    public PhotonVisionHelper getBackCamera(){
        return backCamera;
    }

    public static VisionSubsystem getInstance() {
        if (instance == null) {
          instance = new VisionSubsystem();
        }
    
        return instance;
      }

}
