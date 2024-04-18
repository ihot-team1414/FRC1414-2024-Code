package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.TreeMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.utils.ShooterEntry;

public final class Constants {
    public static final class DriveConstants {

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 6.01;
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

        public static final double kAutoAimTeleopErrorMargin = 4;
        public static final double kAutoAimAutoErrorMargin = 4;
        public static final double kSlowMode = 0.1;

        public static final double kAutoAimP = 3;
        public static final double kAutoAimI = 0.0;
        public static final double kAutoAimD = 0;
        public static final int kPigeonCanID = 60;
        public static final boolean kUsePigeon = false;
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
        public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
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
        public static final double kMaxSpeedMetersPerSecond = 5.5;
    }

    public static final class VisionConstants {
        public static final double kCacheTimeout = 10;

        public static final boolean kShootOnTheMove = true;
        public static final double kEstimatedShotSpeed = 20;
    }

    public static final class VortexMotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotor1CanId = 35;
        public static final int kIntakeMotor2CanId = 34;

        public static final int kIntakeMotorCurrentLimit = 10;

        public static double kIntakeDutyCycle = 0.75;
        public static double kOuttakeDutyCycle = -0.75;
        public static final double kFeedDutyCycle = 0.75;

        public static final int kIntakeSensorMiddleCanId = 51;
        public static final int kIntakeSensorBackCanId = 50;
        public static final int kIntakeSensorFrontCanId = 0; // TODO

        public static double kFrontSensorThreshold = 400;
        public static double kMiddleSensorThreshold = 238;
        public static double kBackSensorThreshold = 238;
    }

    public static final class ShooterConstants {
        public static final int kShooterMotor1CanId = 32;
        public static final int kShooterMotor2CanId = 33;
        public static final int kShooterMotorCurrentLimit = 40;

        public static final Slot0Configs kShooterConfiguration = new Slot0Configs();

        static {
            kShooterConfiguration.kP = 0.80275;
            kShooterConfiguration.kI = 0;
            kShooterConfiguration.kD = 0;

            kShooterConfiguration.kA = 0.037221;
            kShooterConfiguration.kV = 0.1685;
        }

        public static final Measure<Voltage> kAmpVoltage = Volts.of(3);
        public static final Measure<Voltage> kOuttakeVoltage = Volts.of(-4);

        public static final Measure<Voltage> kEjectVoltage = Volts.of(4);
        public static final double kEjectVelocity = 20;

        public static final Measure<Voltage> kSubwooferShotVoltage = Volts.of(8.5);
        public static final double kSubwooferShotVelocity = 60;

        public static final double kShotSpeed = 70;
        public static final double kAutoAimTimeout = 1.5;
        public static final double kPassVelocity = 50;
    }

    public static final class PivotConstants {
        public static final int kPivotMotor1CanId = 30;
        public static final int kPivotMotor2CanId = 31;
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

        public static final double kIntakePosition = 2;
        public static final double kAmpScoringPosition = 17.5;
        public static final double kSubwooferShotPosition = 10;
        public static final double kReverseShotPosition = 18.25;
        public static final double kStowPosition = 0.5;
        public static final double kEjectPosition = 10;
        public static final double kOuttakePosition = 5;
        public static final double kPivotErrorMargin = 0.2;
        public static final double kMinFeedPosition = 4;
    }

    public static final class DeflectorConstants {
        public static final int kDeflectorMotor1CanId = 41;
        public static final int kDeflectorMotor2CanId = 40;
        public static final int kDeflectorCurrentLimit = 5;

        public static final double kDeflectorP = 1;
        public static final double kDeflectorI = 0.0;
        public static final double kDeflectorD = 0.0;

        public static final double kDeflectorScoringPosition = -1.7;
        public static final double kDeflectorStowPosition = 0;

        public static final double kDeflectorTolerance = 0.02;
    }

    public static final class FieldConstants {
        public static final Translation2d bluePassPosition = new Translation2d(0.6, 7.5);
        public static final Translation2d redPassPosition = new Translation2d(16, 7.5);

        public static final Supplier<Translation2d> alliancePassPositionSupplier = () -> DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                        ? FieldConstants.bluePassPosition
                        : FieldConstants.redPassPosition;

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
                put(51, new Translation2d(Units.inchesToMeters(566.77), Units.inchesToMeters(287.00)));
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
                put(61, new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(287)));
            }
        };

        public static Supplier<Translation2d> allianceSpeakerPositionSupplier = () -> DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == Alliance.Blue
                        ? FieldConstants.getTagTranslation(FieldConstants.kBlueSpeakerId)
                        : FieldConstants.getTagTranslation(FieldConstants.kRedSpeakerId);

        public static final Translation2d ampEntryOffset = new Translation2d(0, -2);

        public static final Pose2d ampEntryTolerance = new Pose2d(new Translation2d(1, 1),
                Rotation2d.fromDegrees(2));

        public static final Supplier<Pose2d> allianceAmpEntryPoseSupplier = () -> DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == Alliance.Blue
                        ? new Pose2d(FieldConstants.getTagTranslation(FieldConstants.kBlueAmpId).plus(ampEntryOffset),
                                Rotation2d.fromDegrees(90))
                        : new Pose2d(FieldConstants.getTagTranslation(FieldConstants.kRedAmpId).plus(ampEntryOffset),
                                Rotation2d.fromDegrees(90));

        public static final int kRedSpeakerId = 4;
        public static final int kBlueSpeakerId = 7;
        public static final int kRedAmpId = 5;
        public static final int kBlueAmpId = 6;

        public static Translation2d getTagTranslation(int id) {
            if (kRedAprilTagLayout.containsKey(id)) {
                return kRedAprilTagLayout.get(id);
            } else if (kBlueAprilTagLayout.containsKey(id)) {
                return kBlueAprilTagLayout.get(id);
            } else {
                return null;
            }
        }
    }

    public static final class LEDConstants {
        public static final int kPWMPort = 9;

        public static final double kLEDRed = 0.61;
        public static final double kLEDViolet = 0.91;
        public static final double kLEDBlue = 0.83;
        public static final double kLEDGreen = 0.77;
        public static final double kLEDBlack = 0.99;
        public static final double kLEDOrange = 0.17;

        public static final double kLEDRedBreath = -0.17;

        public static final double kLEDHeartbeatMediumColor1 = 0.05;
        public static final double kLEDHeartbeatMediumColor2 = 0.25;

        public static final double kLEDStrobeRed = -0.11;
        public static final double kLEDStrobeOrange = 0.27;
        public static final double kLEDLightChaseRed = -0.31;
        public static final double kLEDGreenFlashing = 0.15;
        public static final double kDisabledLED = 0.01; // -0.99 is rainbow

        public static final double kLarsonLED = -0.01;
    }

    public static final class ShooterData {
        public static final TreeMap<Double, ShooterEntry> passingData = new TreeMap<Double, ShooterEntry>() {
            {
                put(5.0, new ShooterEntry(12, 70));
                put(7.0, new ShooterEntry(10, 70));
                put(7.5, new ShooterEntry(9.6, 70));
                put(8.0, new ShooterEntry(9.5, 70));
                put(8.5, new ShooterEntry(9.4, 70));
                put(9.0, new ShooterEntry(9.3, 70));
                put(9.5, new ShooterEntry(9.15, 70));
                put(10.0, new ShooterEntry(7, 70));
            }
        };

        public static final TreeMap<Double, ShooterEntry> speakerData = new TreeMap<Double, ShooterEntry>() {
            {
                put(0.0, new ShooterEntry(9.2, 70));
                put(1.46, new ShooterEntry(9, 70)); // 5
                                                    // inches
                                                    // from
                                                    // subwoofer
                put(1.68, new ShooterEntry(8.7, 70)); // 15 inches
                put(1.85, new ShooterEntry(8, 70)); // 20 inches
                put(1.93, new ShooterEntry(7.7, 70)); // 25 inches
                put(2.05, new ShooterEntry(7.35, 70)); // 30 inches
                put(2.18, new ShooterEntry(7, 70));
                put(2.31, new ShooterEntry(6.88, 70));
                put(2.46, new ShooterEntry(6.7, 70));
                put(2.6, new ShooterEntry(6.51, 70));
                put(2.75, new ShooterEntry(6.32, 70));
                put(2.85, new ShooterEntry(6.18, 70));
                put(2.95, new ShooterEntry(6, 70));
                put(3.12, new ShooterEntry(5.83, 70));
                put(3.27, new ShooterEntry(5.68, 70));
                put(3.35, new ShooterEntry(5.56, 70));
                put(3.43, new ShooterEntry(5.53, 70));
                put(3.52, new ShooterEntry(5.3, 70));
                put(3.69, new ShooterEntry(5.1, 70));
                put(3.75, new ShooterEntry(5.075, 70));
                put(3.85, new ShooterEntry(5, 70));
                put(3.88, new ShooterEntry(4.95, 70));
                put(4.0, new ShooterEntry(4.86, 70));
                put(4.1, new ShooterEntry(4.68, 70));
                put(4.25, new ShooterEntry(4.56, 70));
                put(4.33, new ShooterEntry(4.48, 70));
                put(4.39, new ShooterEntry(4.42, 70));
                put(4.54, new ShooterEntry(4.3, 70));
                put(4.62, new ShooterEntry(4.25, 70));
                put(4.78, new ShooterEntry(4.25, 70));
                put(4.86, new ShooterEntry(4.24, 70));
                put(5.0, new ShooterEntry(4.14, 70));
                put(5.25, new ShooterEntry(4.04, 70));
            }
        };

        public static final TreeMap<Double, ShooterEntry> fallbackSpeakerData = new TreeMap<Double, ShooterEntry>() {
            {
                put(0.0, new ShooterEntry(6, 70));
                put(1.75, new ShooterEntry(9.9, 70));
                put(2.0, new ShooterEntry(8.8, 70));
                put(2.1, new ShooterEntry(8.3, 70));
                put(2.25, new ShooterEntry(8, 70));
                put(2.35, new ShooterEntry(7.75, 70));
                put(2.55, new ShooterEntry(7.5, 70));
                put(2.65, new ShooterEntry(7.2, 70));
                put(2.75, new ShooterEntry(7.0, 70));
                put(2.8, new ShooterEntry(6.9, 70));
                put(2.9, new ShooterEntry(6.75, 70));
                put(3.0, new ShooterEntry(6.25, 70));
                put(3.25, new ShooterEntry(6.17, 70));
                put(3.3, new ShooterEntry(6.05, 70));
                put(3.35, new ShooterEntry(6, 70));
                put(3.4, new ShooterEntry(5.90, 70));
                put(3.5, new ShooterEntry(5.85, 70));
                put(3.6, new ShooterEntry(5.75, 70));
                put(3.7, new ShooterEntry(5.65, 70));
                put(3.8, new ShooterEntry(5.6, 70));
                put(3.9, new ShooterEntry(5.55, 70));
                put(3.95, new ShooterEntry(5.5, 70));
                put(4.0, new ShooterEntry(5.4, 70));
                put(4.2, new ShooterEntry(5.4, 70));
                put(4.3, new ShooterEntry(5.3, 70));
                put(4.4, new ShooterEntry(5.25, 70));
                put(4.5, new ShooterEntry(5.25, 70));
                put(4.6, new ShooterEntry(5.2, 70));
                put(4.7, new ShooterEntry(5.2, 70));
                put(4.8, new ShooterEntry(5.15, 70));
                put(4.85, new ShooterEntry(5.25, 70));
                put(5.2, new ShooterEntry(4.8, 70));
                put(5.5, new ShooterEntry(4.6, 70));
                put(5.75, new ShooterEntry(4.6, 70));
                put(6.0, new ShooterEntry(4.45, 70));
                put(6.25, new ShooterEntry(4.4, 70));
                put(6.5, new ShooterEntry(4.3, 70));
            }
        };
    }
}