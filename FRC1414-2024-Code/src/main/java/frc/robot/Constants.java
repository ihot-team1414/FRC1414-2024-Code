// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.TreeMap;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2; //
    public static final double kFrontRightChassisAngularOffset = 0; //
    public static final double kBackLeftChassisAngularOffset = Math.PI; //
    public static final double kBackRightChassisAngularOffset = Math.PI / 2; //

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 12;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 13;

    public static final int kFrontLeftTurningCanId = 20;
    public static final int kRearLeftTurningCanId = 22;
    public static final int kFrontRightTurningCanId = 21;
    public static final int kRearRightTurningCanId = 23;

    public static final boolean kGyroReversed = false;

    public static final double kYawThreshold = 1.5; // for aimToTarget rotation
    public static final double kSlowMode = 0.1;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.075;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionConstants {

    public static final double kFrontCameraHeight = Units.inchesToMeters(7.25);
    public static final double kFrontCameraPitch = Math.toRadians(40);
    public static final Transform3d kFrontCameraToRobot = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(-14.5), 
        Units.inchesToMeters(1), 
        Units.inchesToMeters(2)),
      new Rotation3d(Units.degreesToRadians(5), Units.degreesToRadians(7.5), Units.degreesToRadians(0)));
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotor1CanId = 30;
    public static final int kIntakeMotor2CanId = 31;
    public static final boolean kIntakeMotorInverted = false;
    public static final int kIntakeMotorCurrentLimit = 40;
    public static double kIntakeSpeed = 1;
    public static int kIntakeSensorPort = 50;
    public static double kIndexThreshold = 45;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotor1CanId = 40;
    public static final int kShooterMotor2CanId = 41;
    public static final boolean kShooterMotorInverted = false;
    public static final int kShooterMotorCurrentLimit = 40;

    //Fill
    public static final Slot0Configs kShooterConfiguration = new Slot0Configs();
    public static final double kOuttakeVelocity = 0;
    public static final double kShooterThreshold = 0;
    static {
      kShooterConfiguration.kP = 1;
      kShooterConfiguration.kI = 0;
      kShooterConfiguration.kD = 0;
    }
  }

  public static final class PivotConstants {
    public static final int kPivotMotor1CanId = 50;
    public static final int kPivotMotor2CanId = 51;
    public static final double kMaxAngleThreshold = 0;
    public static final double kMinAngleThreshold = 0;
    public static final Slot0Configs kPivotConfiguration = new Slot0Configs();
    public static double kPivotThreshold = 0;

  }

  public static final class FieldConstants {
    public static final TreeMap<Integer, Translation2d> kRedAprilTagLayout = new TreeMap<Integer, Translation2d>() {
      {
        put(1, new Translation2d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68)));
        put(2, new Translation2d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79)));
        put(3, new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17)));
        put(4, new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)));
        put(5, new Translation2d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00)));
        put(11, new Translation2d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19)));
        put(12, new Translation2d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10)));
        put(13, new Translation2d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62)));
      }
     
    };

    public static final TreeMap<Integer, Translation2d> kBlueAprilTagLayout = new TreeMap<Integer, Translation2d>() {
      {
        put(6, new Translation2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00)));
        put(7, new Translation2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42)));
        put(8, new Translation2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(196.17)));
        put(9, new Translation2d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79)));
        put(10, new Translation2d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68)));
        put(14, new Translation2d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62)));
        put(15, new Translation2d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10)));
        put(16, new Translation2d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19)));
      }
    };

    public static final double originToWing = Units.inchesToMeters(229.19);
    public static final double kBlueWingX = originToWing;
    public static final double kRedWingX = 16.48 - originToWing;

    public static final double kStageHeight = 1.32;
    public static final double kAmpHeight = 1.36;
    public static final double kSpeakerHeight = 1.45;

    public static final int kRedSpeakerID = 3;
    public static final int kBlueSpeakerID = 7; //compare to arraylist
    public static final int kRedAmpID = 5;
    public static final int kBlueAmpID = 6;

    public static final double kDistanceThreshold = 1;

    public static boolean isRedTag(int id) {
      return kRedAprilTagLayout.containsKey(id);
    }

    private static Translation2d getRedTagTranslation(int id) {
      return kRedAprilTagLayout.get(id);
    }

    private static Translation2d getBlueTagTranslation(int id) {
      return kBlueAprilTagLayout.get(id);
    }

    public static Translation2d getTagTranslation(int id) {
      if(kRedAprilTagLayout.containsKey(id)){
        return getRedTagTranslation(id);
      }
      else if(kBlueAprilTagLayout.containsKey(id)){
        return getBlueTagTranslation(id);
      }
      else{
        return null;
      }
    }
  }
}