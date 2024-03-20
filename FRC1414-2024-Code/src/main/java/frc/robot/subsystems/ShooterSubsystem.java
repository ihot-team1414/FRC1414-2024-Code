package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.utils.ShooterData;
import frc.robot.commands.StartShooter;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final Follower follower;
    private final VelocityDutyCycle shooterVelocity;
    private final TalonFXConfiguration shooterMotorConfig;
    private static ShooterSubsystem instance;
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
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        
        shooterMotor1.getConfigurator().apply(shooterMotorConfig);
        shooterMotor2.getConfigurator().apply(shooterMotorConfig);

        follower = new Follower(shooterMotor1.getDeviceID(), true);

        shooterMotor2.setControl(follower);

        shooterMotor1.getConfigurator().refresh(ShooterConstants.kShooterConfiguration);
        shooterMotor2.getConfigurator().refresh(ShooterConstants.kShooterConfiguration);

        //Refresh?
        //Configure PID, max output, voltage compensation
    }

    // Set the speed of the shooter motors to the given velocity from the shooter data
    public void setShooterSpeed(double speed) {
        this.speed = speed;
        shooterMotor1.setControl(shooterVelocity.withVelocity(speed));
    }

    /*
    public double getShooterSpeed(){
        double distance = VisionSubsystem.getInstance().getFrontCam().getDistance();
        return ShooterData.getInstance().getShooterSpeed(distance);
    }

    public double getSafeSpeed(){
        double distance = VisionSubsystem.getInstance().getFrontCam().getDistance();
        return ShooterData.getInstance().getSafeSpeed(distance);
    }*/

    // Check whether the current velocity of the motor is within the threshold (before shooting)
    public boolean isWithinThreshold(){
        return Math.abs(shooterMotor1.getVelocity().getValueAsDouble() - speed) < ShooterConstants.kShooterThreshold;
    }

    // Outtake through the shooter motors
    public void outtake(){
        speed = ShooterConstants.kOuttakeVelocity;
        shooterMotor1.setControl(shooterVelocity.withVelocity(ShooterConstants.kOuttakeVelocity));
    }

    public void shootVolt(){
        shooterMotor1.set(0.2);
    }

    // Stop motors on coast
    public void stop(){
        speed = 0;
        shooterMotor1.stopMotor();
    }

    public void shoot(){
        if(DrivetrainSubsystem.getInstance().inWing())
        { new StartShooter(); }
        else 
        { PivotSubsystem.getInstance().home(); }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter Motor 1 Velocity", shooterMotor1.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Shooter Motor 2 Velocity", shooterMotor2.getVelocity().getValueAsDouble() * 60);
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
          instance = new ShooterSubsystem();
        }
    
        return instance;
      }
}
