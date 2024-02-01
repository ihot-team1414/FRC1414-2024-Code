// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    // Point angles of pivot
    // Need to find values - zero is used as a filler
    public static final double kPivotAngleGround = 0;
    public static final double kPivotAngleStow = 0;

    // Absolute encoder offset
    // Need to find values
    public static final double kPivotEncoderOffset = 0.0;
    public static final int kPivotEncoderId = 0;

    // Intake speeds
    // Need to find values - zero is used as a filler
    public static final double kIntakeEffort = 0;
    public static final double kEjectEffort = 0;
    public static final double kFeedLauncherEffort = 0;
  }

  public static class VisionConstants {
    public static final String CAM1_NAME = "OV9281-1";
    public static final String CAM2_NAME = "OV9281-2";
    public static final String CAM3_NAME = "OV9281-3";
    public static final String CAM4_NAME = "OV9281-4";

    private static final double CAM_HEIGHT = Units.inchesToMeters(16);
    private static final double CAM_X = Units.inchesToMeters(6.6 / 2.0);
    private static final double CAM_Y = Units.inchesToMeters(15.3/2.0);
    private static final double CAM_PITCH = Units.degreesToRadians(-15);
    private static final double CAM_YAW = Units.degreesToRadians(32);

    public static final Transform3d robotToCam1 = new Transform3d(
      new Translation3d(CAM_X, CAM_Y, CAM_HEIGHT), new Rotation3d(0, CAM_PITCH, CAM_YAW));
    public static final Transform3d robotToCam2 = new Transform3d(
      new Translation3d(CAM_X, -CAM_Y- Units.inchesToMeters(0.5), CAM_HEIGHT), 
      new Rotation3d(0, CAM_PITCH, -CAM_YAW));
    public static final Transform3d robotToCam3 = new Transform3d(new Translation3d(-CAM_X, CAM_Y, CAM_HEIGHT), 
      new Rotation3d(0, CAM_PITCH, (Math.PI) - CAM_YAW));
    public static final Transform3d robotToCam4 = new Transform3d(
      new Translation3d(-CAM_X, -CAM_Y, CAM_HEIGHT), 
      new Rotation3d(0, CAM_PITCH, (Math.PI) + CAM_YAW + Units.degreesToRadians(5.5)));
    
    public static AprilTagFieldLayout TAG_FIELD_LAYOUT = 
    new AprilTagFieldLayout(
      List.of(
        // Red Alliance Community (Right to Left)
        new AprilTag(1, new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d(0, 0, Math.PI))),
        new AprilTag(2, new Pose3d(15.513558, 2.748026, 0.462788, new Rotation3d(0, 0, Math.PI))),
        new AprilTag(3, new Pose3d(15.513558, 4.424426, 0.462788, new Rotation3d(0, 0, Math.PI))),
        // Blue Alliance Double Substation
        new AprilTag(4, new Pose3d(16.178784, 6.749796, 0.695452, new Rotation3d(0, 0, Math.PI))),
        // Red Alliance Double Substation
        new AprilTag(5, new Pose3d(0.36195, 6.749796, 0.695452, new Rotation3d(0, 0, 0))),
        // Blue Alliance Community (Right to Left)
        new AprilTag(6, new Pose3d(1.02743, 4.424426, 0.462788, new Rotation3d(0, 0, 0))),
        new AprilTag(7, new Pose3d(1.02743, 2.748026, 0.462788, new Rotation3d(0, 0, 0))),
        new AprilTag(8, new Pose3d(1.02743, 1.071626, 0.462788, new Rotation3d(0, 0, 0)))),
        16.54175,
        8.0137);
  }
}
