package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final Follower follower;
    private final VelocityDutyCycle shooterVelocity;
    private final TalonFXConfiguration shooterMotorConfig;
    private double speed;

    public ShooterSubsystem() {

        shooterMotor1 = new TalonFX(ShooterConstants.kShooterMotor1CanId);
        shooterMotor2 = new TalonFX(ShooterConstants.kShooterMotor2CanId);
        shooterVelocity = new VelocityDutyCycle(0);
        speed = 0;

        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor2.setNeutralMode(NeutralModeValue.Coast);

        shooterMotorConfig = new TalonFXConfiguration();
        shooterMotorConfig.withSlot0(ShooterConstants.kShooterConfiguration);
        
        shooterMotor1.getConfigurator().apply(shooterMotorConfig);
        follower = new Follower(shooterMotor1.getDeviceID(), true);

        shooterMotor2.setControl(follower);

        shooterMotor1.getConfigurator().refresh(ShooterConstants.kShooterConfiguration);

        //Refresh?
        //Configure PID, max output, voltage compensation
    }

    // Set the speed of the shooter motors to the given velocity from the shooter data
    public void setShooterSpeed(double speed) {
        this.speed = speed;
        shooterMotor1.setControl(shooterVelocity.withVelocity(speed));
    }

    // Check whether the current velocity of the motor is within the threshold (before shooting)
    public boolean isWithinThreshold(){
        return Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - speed) < ShooterConstants.kShooterThreshold;
    }

    // Outtake through the shooter motors
    public void outtake(){
        shooterMotor1.setControl(shooterVelocity.withVelocity(ShooterConstants.kOuttakeVelocity));
    }

    // Stop motors on coast
    public void stop(){
        shooterMotor1.stopMotor();
    }
  
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter Motor 1 Velocity", shooterMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor 2 Velocity", shooterMotor2.getVelocity().getValueAsDouble());
    }
}
