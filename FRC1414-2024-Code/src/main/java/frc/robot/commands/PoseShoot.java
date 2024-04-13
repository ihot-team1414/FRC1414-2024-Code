package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.OdometryData;
import frc.utils.RobotState;
import frc.utils.RobotState.RobotConfiguration;

import java.util.function.DoubleSupplier;

public class PoseShoot extends Command {

    private final DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();
    private final PivotSubsystem pivot = PivotSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private boolean isRed;

    private final PIDController rotController = new PIDController(4, 0, 0);

    private Translation2d target;

    public PoseShoot(
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            Translation2d target,
            boolean isRed) {

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.isRed = isRed;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        rotController.setTolerance(1);
        rotController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {

        //Turn to point calculation
        double xPose = drive.getCurrentPose().getX();
        double yPose = drive.getCurrentPose().getY();

        double xAngle = xPose - target.getX();
        double yAngle = yPose - target.getY();

        //Vector distance calculation
        Vector<N2> robot = VecBuilder.fill(xPose, yPose);
        Vector<N2> end = VecBuilder.fill(target.getX(), target.getY());
        
        double distance = (robot.minus(end)).norm();

        double angle = drive.getHeading().getDegrees();
        double additive = xAngle < 0 ? 180 : 0;
        double difference = Math.toDegrees(Math.atan(yAngle / xAngle)) + additive;
        rotController.setSetpoint(difference);

        Rotation2d rotation = Rotation2d.fromDegrees(-rotController.calculate(-angle, -rotController.getSetpoint()));

        double translationX = translationXSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;
        double translationY = translationYSupplier.getAsDouble()
                * DriveConstants.kMaxSpeedMetersPerSecond;

        drive.drive(new Transform2d(new Translation2d(translationX, translationY),
                        rotation),
                        true);


        double heading = drive.getDegrees();
        double position;
        if (heading >= 45) { target = isRed ? FieldConstants.kSector.get("RAmp") : FieldConstants.kSector.get("BAmp");
            position = OdometryData.getInstance().getShooterPosition(distance, "AMP"); }
        else if (heading <= -45) { target = isRed ? FieldConstants.kSector.get("RWeak") : FieldConstants.kSector.get("BWeak");
            position = OdometryData.getInstance().getShooterPosition(distance, "WEAK"); }
        else { target = isRed ? FieldConstants.kSector.get("RCenter") : FieldConstants.kSector.get("BCenter");
            position = OdometryData.getInstance().getShooterPosition(distance); }

        pivot.setPosition(position);
        shooter.setVelocity(ShooterConstants.kShotSpeed);

        if(rotController.atSetpoint() 
            && shooter.isWithinVelocityTolerance(ShooterConstants.kPassVelocity)
            && pivot.isAtPositionSetpoint(position))
            {
                intake.setDutyCycle(IntakeConstants.kSpeakerFeedDutyCycle);
                RobotState.getInstance().setRobotConfiguration(RobotConfiguration.SHOOTING);
            }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        intake.stop();
        pivot.setPosition(PivotConstants.kStowPosition);
        drive.lock();
        RobotState.getInstance().setRobotConfiguration(RobotConfiguration.STOWED);
    }
}