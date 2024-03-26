package frc.robot;

import java.util.TreeMap;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 12;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 13;

    public static final int kFrontLeftTurningCanId = 20;
    public static final int kRearLeftTurningCanId = 22;
    public static final int kFrontRightTurningCanId = 21;
    public static final int kRearRightTurningCanId = 23;

    public static final double kAutoAimTeleopErrorMargin = 3.5;
    public static final double kAutoAimAutoErrorMargin = 1;
    public static final double kSlowMode = 0.1;

    public static final double kAutoAimP = 5;
    public static final double kAutoAimI = 0;
    public static final double kAutoAimD = 0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 21) / (kDrivingMotorPinionTeeth * 15);
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
    public static final int kOperatorControllerPort = 1;
    public static final double kJoystickDeadband = 0.075;
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
    public static final double kLimelightAngle = 25; // from vertical
    public static final double kLimelightHeight = 10; // inches

    public static final double kCacheTimeout = 45; // seconds

    public static final double kRightLimelightOffset = -30;
    public static final double kLeftLimelightOffset = 30;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotor1CanId = 35;
    public static final int kIntakeMotor2CanId = 34;
    public static final boolean kIntakeMotorInverted = false;
    public static final int kIntakeMotorCurrentLimit = 40;
    public static final double kSpeakerFeedDutyCycle = 0.75;
    public static final double kAmpFeedDutyCycle = 0.6;
    public static final Slot0Configs kIntakeConfiguration = new Slot0Configs();
    static {
      kIntakeConfiguration.kP = 1;
      kIntakeConfiguration.kI = 0;
      kIntakeConfiguration.kD = 0;
    }
    public static double kIntakeDutyCycle = 0.75;
    public static int kIntakeSensorCandId = 50;
    public static double kIndexThreshold = 228;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotor1CanId = 32;
    public static final int kShooterMotor2CanId = 33;
    public static final int kShooterMotorCurrentLimit = 40;

    public static final Slot0Configs kShooterConfiguration = new Slot0Configs();

    static {
      kShooterConfiguration.kP = 0.20275;
      kShooterConfiguration.kI = 0;
      kShooterConfiguration.kD = 0;

      kShooterConfiguration.kA = 0.037221;
      kShooterConfiguration.kV = 0.1685;
    }

    public static final double kAmpDutyCycleLeft = 0.3225;
    public static final double kAmpDutyCycleRight = 0.1225;
    public static final double kEjectDutyCycle = 0.4;
    public static final double kRestDutyCycle = 0.1;
    public static final double kShooterErrorMargin = 10;
    public static final double kSpeakerShotDutyCycle = 0.7;
    public static final double kShotSpeedDutyCycle = 0.7;

    public static final double kShooterRestSpeed = 5; // TODO
    public static final double kShotSpeed = 10; // TODO
  }

  public static final class PivotConstants {
    public static final int kPivotMotor1CanId = 30;
    public static final int kPivotMotor2CanId = 31;
    public static final double kMaxAngleThreshold = 18;
    public static final double kMinAngleThreshold = 1;
    public static final Slot0Configs kPivotConfiguration = new Slot0Configs();

    static {
      kPivotConfiguration.kP = 0.75;
      kPivotConfiguration.kI = 0;
      kPivotConfiguration.kD = 0.001;
    }

    public static final MotionMagicConfigs kPivotMotionMagic = new MotionMagicConfigs();

    static {
      kPivotMotionMagic.MotionMagicCruiseVelocity = 200;
      kPivotMotionMagic.MotionMagicAcceleration = 100;
    }

    public static final double kIntakePosition = 6; // 2
    public static final double kAmpPrimePosition = 6;
    public static final double kAmpScoringPosition = 17.5;
    public static final double kSpeakerShotPosition = 10;
    public static final double kStowPosition = 6; // 0.5
    public static final double kEjectPosition = 6;
    public static final double kPivotErrorMargin = 0.2;
    public static final double kAmpFeedPosition = 7.4; // 7.4
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

    public static final Translation2d kBlueAmpSafe = new Translation2d(Units.inchesToMeters(72.5),
        Units.inchesToMeters(313.00));
    public static final Translation2d kRedAmpSafe = new Translation2d(Units.inchesToMeters(587.77),
        Units.inchesToMeters(313.00));

    public static final double originToWing = Units.inchesToMeters(229.19);
    public static final double kBlueWingX = originToWing;
    public static final double kRedWingX = 16.48 - originToWing;

    public static final double kStageHeight = 1.32;
    public static final double kAmpHeight = 1.36;
    public static final double kSpeakerHeight = 1.45;

    public static final int kRedSpeakerID = 3;
    public static final int kBlueSpeakerID = 7; // compare to arraylist
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
      if (kRedAprilTagLayout.containsKey(id)) {
        return getRedTagTranslation(id);
      } else if (kBlueAprilTagLayout.containsKey(id)) {
        return getBlueTagTranslation(id);
      } else {
        return null;
      }
    }
  }

  public static final class LEDConstants {

    public static final int kPWMPort = 0;

    public static final double kLEDRed = 0.61;
    public static final double kLEDGold = 0.67;
    public static final double kLEDBlue = 0.87;
    public static final double kLEDGreen = 0.73;
    public static final double kLEDBlack = 0.99;
    public static final double kLEDOrange = 0.65;

    public static final double kLEDRedBreath = -0.17;

    public static final double kLEDHeartbeatMediumColor1 = 0.05;
    public static final double kLEDHeartbeatMediumColor2 = 0.25;

    public static final double kLEDStrobeRed = -0.11;
    public static final double kLEDLightChaseRed = -0.31;
    public static final double kLEDGreenFlashing = -0.61;
    public static final double kLEDRainbow = -0.99;

  }
}