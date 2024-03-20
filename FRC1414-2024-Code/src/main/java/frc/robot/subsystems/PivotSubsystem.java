package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
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
    private final MotionMagicDutyCycle pivotPosition;
    private final TalonFXConfiguration pivotMotorConfig;
    private static PivotSubsystem instance;
    private double position;

    public PivotSubsystem() {
            
        pivotMotor1 = new TalonFX(PivotConstants.kPivotMotor1CanId);
        
        pivotMotor2 = new TalonFX(PivotConstants.kPivotMotor2CanId);
        pivotPosition = new MotionMagicDutyCycle(0, true, 0, 0, false, false, false);
        position = 0;

        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor2.setNeutralMode(NeutralModeValue.Brake);

        pivotMotor1.setPosition(0);
        pivotMotor2.setPosition(0);

        pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.withSlot0(PivotConstants.kPivotConfiguration);
        pivotMotorConfig.withMotionMagic(PivotConstants.kPivotMotionMagic);

        pivotMotor1.getConfigurator().apply(pivotMotorConfig);
        pivotMotor2.getConfigurator().apply(pivotMotorConfig);
        follower = new Follower(pivotMotor1.getDeviceID(), true);

        pivotMotor2.setControl(follower);

        pivotMotor1.getConfigurator().refresh(pivotMotorConfig); 
        pivotMotor2.getConfigurator().refresh(pivotMotorConfig);

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
        double distance = VisionSubsystem.getInstance().getFrontCam().getDistance();
        position = ShooterData.getInstance().getShooterAngle(distance);
        return position;
    }

    public double getSafeAngle(){
        double distance = VisionSubsystem.getInstance().getFrontCam().getDistance();
        return ShooterData.getInstance().getSafeAngle(distance);
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
        position = 1;
        setPivotAngle(PivotConstants.kMinAngleThreshold);
    }

    public void getPosition(){
        pivotMotor1.getPosition().getValueAsDouble();
    }

    // Check whether the current velocity of the motor is within the threshold (before shooting)
    public boolean isWithinThreshold(){
        return Math.abs(pivotMotor1.getPosition().getValueAsDouble() - position) < PivotConstants.kPivotThreshold;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Motor Angle 1", pivotMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Motor Angle 2", pivotMotor2.getPosition().getValueAsDouble());
    }
}
