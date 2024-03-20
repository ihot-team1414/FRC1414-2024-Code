package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;;

public class IntakeSubsystem extends SubsystemBase {
    
    private final TalonFX intakeMotor1;
    private final TalonFX intakeMotor2;
    private final TimeOfFlight intakeSensor;
    private Follower followerI; 
    private DutyCycleOut dutyCycleOut;
    private TalonFXConfiguration intakeMotorConfiguration;

    private static IntakeSubsystem instance;

    public IntakeSubsystem(){
        
        intakeMotor1 = new TalonFX(IntakeConstants.kIntakeMotor1CanId);
        intakeMotor2 = new TalonFX(IntakeConstants.kIntakeMotor2CanId);
        intakeSensor = new TimeOfFlight(IntakeConstants.kIntakeSensorPort);
        
        intakeMotor1.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor2.setNeutralMode(NeutralModeValue.Coast);
        intakeSensor.setRangingMode(RangingMode.Short, 32);

        intakeMotorConfiguration = new TalonFXConfiguration();
        intakeMotorConfiguration.withSlot0(IntakeConstants.kIntakeConfiguration);
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 10;
        
        intakeMotor1.getConfigurator().apply(intakeMotorConfiguration);
        intakeMotor2.getConfigurator().apply(intakeMotorConfiguration);

        followerI = new Follower(intakeMotor1.getDeviceID(), true);

        dutyCycleOut = new DutyCycleOut(0, true, false, false, false);
        intakeMotor2.setControl(followerI);
    }

    public static synchronized IntakeSubsystem getInstance(){
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    //Use exclusively to intake
    public void intake(){
        intakeMotor1.setControl(dutyCycleOut.withOutput(IntakeConstants.kIntakeSpeed));
    }

    //Use exclusively to move from intake to shooter
    public void funnel(){
        intakeMotor1.setControl(dutyCycleOut.withOutput(IntakeConstants.kFunnelSpeed));
    }

    public void outtake(){
        intakeMotor1.setControl(dutyCycleOut.withOutput(-IntakeConstants.kIntakeSpeed));
    }

    public void stop(){
        intakeMotor1.stopMotor();
    }

    public boolean isLoaded(){
        double distance = intakeSensor.getRange();
        return intakeSensor.isRangeValid() ? distance < IntakeConstants.kIndexThreshold : false;
    }

    public void index(){
        //Will keep intaking until loaded
        if(isLoaded())
        { stop(); }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IM1 Velocity", intakeMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("IM2 Velocity", intakeMotor2.getVelocity().getValueAsDouble());
    }
}
