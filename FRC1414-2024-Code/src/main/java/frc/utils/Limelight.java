package frc.utils;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class Limelight {

  private NetworkTableEntry ty = null;
  private NetworkTableEntry tx = null;
  private NetworkTableEntry tv = null;
  private int id;
  private NetworkTable table;
  private double[] configs;
  private Optional<Alliance> allianceColor;

  public Limelight(String networkTable, double[] configs) {

    this.configs = configs;
    NetworkTable table = NetworkTableInstance.getDefault().getTable(networkTable);
    allianceColor = DriverStation.getAlliance();
    id = 0;

    ty = table.getEntry("ty");
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");
  }

  public double getDeltaX() {
    tx = table.getEntry("tx");
    if (tx != null) {
      return tx.getDouble(0.0);
    }
    return 0.0;
  }

  public double getDistance(){
    
    ty = table.getEntry("ty");

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = configs[1]; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = configs[0]; 

    // distance from the target to the floor
    double goalHeightInches = getTargetHeight(); 

    double angleToGoalRadians = Units.degreesToRadians(limelightMountAngleDegrees);

    //calculate distance
    return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }

  public double getTargetHeight(){
    if(detectsTarget()){    
      id = tv.getNumber(0).intValue();
        if(id >= 11) { return FieldConstants.kStageHeight; }
        else if (id == 3 || id == 4 || id == 7 || id == 8) { return FieldConstants.kSpeakerHeight; }
        else { return FieldConstants.kAmpHeight; } 
    }
    return 0.0;
  }

  public boolean targetValid(int id){
      if(detectsTarget() && allianceColor.isPresent()){
          int tag = tv.getNumber(0).intValue();
            if(allianceColor.get().equals(Alliance.Red) && tag == (id) || 
              allianceColor.get().equals(Alliance.Blue) && tag == (id)){
              return true;
          }
      }
      return false;
  }

  public boolean detectsTarget() {
    if (tv != null) {
      return tv.getNumber(0).intValue() > 0;
    }
    return false;
  }

  public double getDeltaY() {
    return ty.getDouble(0.0);
  }

}