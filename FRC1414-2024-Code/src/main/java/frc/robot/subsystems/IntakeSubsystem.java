package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // create single instance of class
    private static IntakeSubsystem instance;

    // idk the CAN ids
    private static int kIntakeMotorCanId;
    private static int kPivotMotorCanId;

    // PID values of the motor pivoting the intake
    // Need to find values - use zero as filler
    private static final double kPivotMotorP = 0.0;
    private static final double kPivotMotorI = 0.0;
    private static final double kPivotMotorD = 0.0;

    private final PIDController mPivotPID = new PIDController(kPivotMotorP, kPivotMotorI, kPivotMotorD);

    private final DutyCycleEncoder mPivotEncoder = new DutyCycleEncoder(IntakeConstants.kPivotEncoderId);

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

    public double getPivotTargetAngle(String target) {
        // return a double based on the target state
        switch (target) {
        case "ground":
            return IntakeConstants.kPivotAngleGround;
        case "stow":
            return IntakeConstants.kPivotAngleStow;
        default:
            // "Safe" default
            return 180;
        }
    }

    public double getIntakeStateEffort(String state) {
        // return a variable based on what state of intake is needed
        switch (state) {
        case "intake":
            return IntakeConstants.kIntakeEffort;
        case "eject":
            return IntakeConstants.kEjectEffort;
        case "feed shooter":
            return IntakeConstants.kFeedShooterEffort;
        default:
            // make the motors not rotate as a default
            return 0.0;
        }
    }

    // make intake spin
    public void setIntakeMotorEffort(double effort) {
        intakeMotor1.set(effort);
    }
    
    /*private static class defaultPosition() {
        // make pivot target "stow"
        // make intake state "empty"

        // create intakePower variable and set to 0.0
        // create pivotPower variable and set to 0.0
    }*/

    public double getPivotAngleDegrees() {
        double positionValue = mPivotEncoder.getAbsolutePosition() - IntakeConstants.kPivotEncoderOffset;
        
        return Units.rotationsToDegrees(positionValue);
    }

    public boolean getHasNote() {
        return intakeMotor1.getOutputCurrent() > noteMinCurrent;
    }

    @Override
    public void periodic() {

    }

    /*@Override
    */
}