package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterData;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.ShooterDataUtils;
import frc.utils.ShooterEntry;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class DeprecatedAutoShootTeleop extends Command {
        private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
        private final PivotSubsystem pivot = PivotSubsystem.getInstance();
        private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
        private final DoubleSupplier translationXSupplier;
        private final DoubleSupplier translationYSupplier;
        private final DoubleSupplier limitingFactorSupplier;

        private double target;
        private double startTime;

        private final PIDController alignmentController = new PIDController(DriveConstants.kAutoAimP,
                        DriveConstants.kAutoAimI, DriveConstants.kAutoAimD);

        public DeprecatedAutoShootTeleop(
                        DoubleSupplier translationXSupplier,
                        DoubleSupplier translationYSupplier,
                        DoubleSupplier limitingFactorSupplier) {

                this.translationXSupplier = translationXSupplier;
                this.translationYSupplier = translationYSupplier;
                this.limitingFactorSupplier = limitingFactorSupplier;
                addRequirements(drivetrain, intake, pivot, shooter);
        }

        @Override
        public void initialize() {
                double yawError = VisionSubsystem.getInstance().getTX().orElse(0.0); //  orElse returns the specified default value, which is 0.0 in this case, if Optional returns empty (equivalent to nulll)

                target = -yawError;

                alignmentController.setTolerance(DriveConstants.kAutoAimTeleopErrorMargin);
                alignmentController.enableContinuousInput(-180, 180);
                SmartDashboard.putNumber("Turning P", DriveConstants.kAutoAimP);
                SmartDashboard.putNumber("Turning I", DriveConstants.kAutoAimI);
                SmartDashboard.putNumber("Turning D", DriveConstants.kAutoAimD);
                alignmentController.setSetpoint(target);

                startTime = Timer.getFPGATimestamp();
        }

        @Override
        public void execute() {

                target = -VisionSubsystem.getInstance().getTX().orElse(0.0);
                double angle = drivetrain.getHeading().getDegrees();

                Rotation2d rotation = Rotation2d.fromDegrees(-alignmentController.calculate(-angle, target));

                double translationX = translationXSupplier.getAsDouble() * limitingFactorSupplier.getAsDouble()
                                * DriveConstants.kMaxSpeedMetersPerSecond;
                double translationY = translationYSupplier.getAsDouble() * limitingFactorSupplier.getAsDouble()
                                * DriveConstants.kMaxSpeedMetersPerSecond;

                Optional<Double> distance = VisionSubsystem.getInstance().getDistance();

                ShooterEntry shooterEntry = ShooterDataUtils.getInterpolatedEntry(ShooterData.fallbackSpeakerData,
                                distance.orElse(0.0));

                pivot.setPosition(shooterEntry.getPosition());
                shooter.setVelocity(ShooterConstants.kShotSpeed);
                // shooter.setDutyCycle(ShooterConstants.kShotSpeedDutyCycle);
                // shooter.setVoltage(ShooterConstants.kShotSpeedDutyCycle);

                double currentTime = Timer.getFPGATimestamp();

                SmartDashboard.putBoolean("Drive Aligned",
                                Math.abs(target + angle) < DriveConstants.kAutoAimTeleopErrorMargin);
                SmartDashboard.putBoolean("Shooter Up To Speed",
                                shooter.isWithinVelocityTolerance(ShooterConstants.kShotSpeed));
                SmartDashboard.putBoolean("Pivot Aligned",
                                pivot.isAtPositionSetpoint(
                                                shooterEntry.getPosition()));

                if ((Math.abs(target + angle) < DriveConstants.kAutoAimTeleopErrorMargin
                                && pivot.isAtPositionSetpoint(
                                                shooterEntry.getPosition())
                                && shooter.isWithinVelocityTolerance(ShooterConstants.kShotSpeed)
                                || (currentTime - startTime > ShooterConstants.kAutoAimTimeout))
                                && VisionSubsystem.getInstance().getDistance().isPresent()
                //
                ) {
                        intake.setDutyCycle(IntakeConstants.kFeedDutyCycle);
                } else {
                        // intake.stop();
                }

                drivetrain
                                .drive(new Transform2d(new Translation2d(translationX, translationY),
                                                rotation),
                                                true);
        }

        @Override
        public void end(boolean interrupted) {
                drivetrain.lock();
                intake.stop();
                shooter.stop();
                pivot.setPosition(Constants.PivotConstants.kStowPosition);
        }
}