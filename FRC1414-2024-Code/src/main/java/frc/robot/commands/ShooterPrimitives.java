package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.LimelightHelpers;
import frc.utils.ShooterData;

public class ShooterPrimitives {
    private static ShooterSubsystem shooter = ShooterSubsystem.getInstance();


    public static Command rev(double dutyCycleLeft, double dutyCycleRight) {
        return new InstantCommand(() -> shooter.setDutyCycle(dutyCycleLeft, dutyCycleRight), shooter);
    }

    public static Command rev(double dutyCycle) {
        return new InstantCommand(() -> shooter.setDutyCycle(dutyCycle), shooter);
    }

    public static Command aim(){
        return new RunCommand(() -> shooter.turnToTarget(), shooter);
    }

    public static Command warmUp(){
        Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front");
        double distance = tagPose.getTranslation().getNorm();
        return new RunCommand(() -> shooter.setDutyCycle(ShooterData.getInstance().getShooterDutyCycle(distance))).finallyDo(() -> shooter.stop());
    }
}
