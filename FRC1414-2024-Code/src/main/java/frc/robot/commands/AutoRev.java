package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.LimelightHelpers;
import frc.utils.ShooterData;

public class AutoRev extends Command {
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public AutoRev() {
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("limelight-front")) {
            Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front");
            double distance = tagPose.getTranslation().getNorm();

            shooter.setDutyCycle(ShooterData.getInstance().getShooterDutyCycle(distance));

            SmartDashboard.putNumber("Tag Distance", tagPose.getTranslation().getNorm());
        } else {
            shooter.setDutyCycle(
                    ShooterData.getInstance().getShooterDutyCycle(Constants.ShooterConstants.kSpeakerShotDutyCycle));
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}