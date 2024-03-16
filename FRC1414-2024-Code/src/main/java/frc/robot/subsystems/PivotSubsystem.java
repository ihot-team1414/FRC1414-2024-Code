package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.ShooterData;

public class PivotSubsystem extends SubsystemBase {

    private final TalonFX pivotMotor1;
    private final TalonFX pivotMotor2;
    private final Follower follower;
    private final PositionDutyCycle pivotPosition;
    private final TalonFXConfiguration pivotMotorConfig;
    private static PivotSubsystem instance;
    private double position;

    public PivotSubsystem() {
            
        pivotMotor1 = new TalonFX(PivotConstants.kPivotMotor1CanId);
        pivotMotor2 = new TalonFX(PivotConstants.kPivotMotor2CanId);
        pivotPosition = new PositionDutyCycle(0);
        position = 0;

        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor2.setNeutralMode(NeutralModeValue.Brake);

        pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.withSlot0(PivotConstants.kPivotConfiguration);

        pivotMotor1.getConfigurator().apply(pivotMotorConfig);
        follower = new Follower(pivotMotor1.getDeviceID(), true);

        pivotMotor2.setControl(follower);

        pivotMotor1.getConfigurator().refresh(PivotConstants.kPivotConfiguration);

        //Refresh?
        //Configure PID, max output, voltage compensation
    }

    public static synchronized PivotSubsystem getInstance(){
        if (instance == null) {
            instance = new PivotSubsystem();
        }
        return instance;
    }

    public double getPivotAngle() {
        double distance = VisionSubsystem.getInstance().getFrontCamera().getDistance();
        return ShooterData.getInstance().getShooterAngle(distance);
    }

    public double getSafeAngle(){
        double distance = VisionSubsystem.getInstance().getFrontCamera().getDistance();
        return ShooterData.getInstance().getShooterAngle(distance);
    }

    // Angle will be filled by getPivotAngle()
    public void setPivotAngle(double position) {

        this.position = position;
        
        if(position > PivotConstants.kMaxAngleThreshold){
            position = PivotConstants.kMaxAngleThreshold;
        }
        else if(position < PivotConstants.kMinAngleThreshold){
            position = PivotConstants.kMinAngleThreshold;
        }
        //Set motors to angle
        pivotMotor1.setControl(pivotPosition.withPosition(position));
    }

    public void home(){
        position = 0;
        setPivotAngle(PivotConstants.kMinAngleThreshold);
    }

    public void getPosition(){
        pivotMotor1.getPosition().getValueAsDouble();
    }

    // Check whether the current velocity of the motor is within the threshold (before shooting)
    public boolean isWithinThreshold(){
        return Math.abs(pivotMotor1.getPosition().getValueAsDouble() - position) < ShooterConstants.kShooterThreshold;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Motor Angle 1", pivotMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Motor Angle 2", pivotMotor2.getPosition().getValueAsDouble());
    }
}
