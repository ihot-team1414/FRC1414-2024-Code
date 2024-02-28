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

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase{
    
    private static VisionSubsystem instance;

    PhotonVision at1 = new PhotonVision("at1");
    PhotonVision at2 = new PhotonVision("at2");

    @Override
    public void periodic(){
      
    }

    public static VisionSubsystem getInstance() {
        if (instance == null) {
          instance = new VisionSubsystem();
        }
    
        return instance;
      }

}
