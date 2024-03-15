package frc.robot.subsystems;

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
import frc.robot.Constants.IntakeConstants;;

public class IntakeSubsystem extends SubsystemBase {
    
    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;
    private final RelativeEncoder intakeMotorEncoder;
    private final SparkPIDController intakeMotorPID;
    private final TimeOfFlight intakeSensor;

    private static IntakeSubsystem instance;

    public IntakeSubsystem(){
        
        intakeMotor1 = new CANSparkMax(IntakeConstants.kIntakeMotor1CanId, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(IntakeConstants.kIntakeMotor2CanId, MotorType.kBrushless);
        intakeSensor = new TimeOfFlight(IntakeConstants.kIntakeSensorPort);
        
        intakeMotor1.setIdleMode(IdleMode.kBrake);
        intakeMotor2.setIdleMode(IdleMode.kBrake);
        intakeSensor.setRangingMode(RangingMode.Short, 32);
        
        intakeMotor1.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        intakeMotor2.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        intakeMotor2.follow(intakeMotor1, true);
        intakeMotorEncoder = intakeMotor1.getEncoder();
        intakeMotorPID = intakeMotor1.getPIDController();
        intakeMotorPID.setFeedbackDevice(intakeMotorEncoder);

        intakeMotor1.burnFlash();
        intakeMotor2.burnFlash();
    }

    public static synchronized IntakeSubsystem getInstance(){
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    //Use exclusively to intake
    public void intake(){
        intakeMotor1.set(IntakeConstants.kIntakeSpeed);
    }

    //Use exclusively to move from intake to shooter
    public void funnel(){
        intakeMotor1.set(IntakeConstants.kFunnelSpeed);
    }

    public void outtake(){
        intakeMotor1.set(-IntakeConstants.kIntakeSpeed);
    }

    public void stop(){
        intakeMotor1.stopMotor();
    }

    public void setSpeed(double speed){
        intakeMotorPID.setReference(speed, ControlType.kVelocity);
    }

    public boolean isLoaded(){
        double distance = intakeSensor.getRange();
        return intakeSensor.isRangeValid() ? distance < IntakeConstants.kIndexThreshold : false;
    }

    public void index(){
        
        //Will keep intaking until loaded
        if(isLoaded())
        { stop(); }
        else 
        { index(); }
   
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IM1 Velocity", intakeMotor1.getEncoder().getVelocity());
        SmartDashboard.putNumber("IM2 Velocity", intakeMotor2.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Filled", isLoaded());
    }
}
