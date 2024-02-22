package frc.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

  private static Limelight instance = null;
  private NetworkTableEntry ty = null;
  private NetworkTableEntry tx = null;
  private NetworkTableEntry tv = null;

  private Limelight() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-b");

    ty = table.getEntry("ty");
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");
  }

  public static Limelight getInstance() {
    if (instance == null) {
      instance = new Limelight();
    }

    return instance;
  }

  public double getDeltaX() {
    if (tx != null) {
      return tx.getDouble(0.0);
    }

    return 0.0;
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