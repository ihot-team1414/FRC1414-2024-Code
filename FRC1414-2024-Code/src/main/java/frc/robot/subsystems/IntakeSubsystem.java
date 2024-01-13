package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // create single instance of class
    private static IntakeSubsystem instance;

    private String pivotTarget;

    // idk the CAN ids
    private static int kIntakeMotorCanId;
    private static int kPivotMotorCanId;

    // PID values of the motor pivoting the intake
    private static final double kPivotMotorP;
    private static final double kPivotMotorI;
    private static final double kPivotMotorD;

    private final PIDController mPivotPID = new PIDController(kPivotMotorP, kPivotMotorI, kPivotMotorD);
    
    // private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.Intake.kPivotEncoderId);
    private final DigitalInput intakeLimitSwitch = new DigitalInput(Constants.Intake.kIntakeLimitSwitchId);

    // some number
    private static int noteMinCurrent;

    // make new motors (NEO 550) to control the intake rollers and the intake as a whole
    private CANSparkMax intakeMotor1 = new CANSparkMax(kIntakeMotorCanId, MotorType.kBrushless);
    private CANSparkMax pivotMotor1 = new CANSparkMax(kPivotMotorCanId, MotorType.kBrushless);

    // maintain that there is one instance of the subsystem class
    public static synchronized IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    private IntakeSubsystem() {
        intakeMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor1.setSmartCurrentLimit(40, 40);

        pivotMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotMotor1.setSmartCurrentLimit(10, 10);
    }

    /*// make spin
    public void setMotorEffort(double effort) {
        intakeMotor1.set(effort);
    }

    // make it stop
    public void stop() {
        intakeMotor1.stop();
    }*/

    public double pivotTargetAngle(String target) {
        // return a double based on the target state
        switch (target) {
        case "ground":
            return Constants.Intake.kPivotAngleGround;
        case "stow":
            return Constants.Intake.kPivotAngleStow;
        default:
            // "Safe" default
            return 180;
        }
    }

    public double intakeStateSpeed(String state) {
        // return a variable based on what state of intake is needed
        switch (state) {
        case "intake":
            return Constants.Intake.kIntakeSpeed;
        case "eject":
            return Constants.Intake.kEjectSpeed;
        case "feed shooter":
            return Constants.Intake.kFeedShooterSpeed;
        default:
            // make the motors not rotate as a default
            return 0.0;
        }
    }

    private static class defaultPosition() {
        // make pivot target "stow"
        // make intake state "empty"

        // create intakePower variable and set to 0.0
        // create pivotPower variable and set to 0.0
    }

    public double getPivotAngleDegrees() {
        double positionValue = mPivotEncoder.getAbsolutePosition() - Constants.Intake.kPivotEncoderOffset;
        
        return Units.rotationsToDegrees(positionValue);
    }

    public boolean getHasNote() {
        return intakeMotor1.getOutputCurrent() > noteMinCurrent;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void stop() {
        intakeMotor1.stop();
        pivotMotor1.stop();
    }
}