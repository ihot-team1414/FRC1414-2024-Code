// now rendered null given robot design changes
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // create single instance of class
    private static IntakeSubsystem instance;

    private static DefaultStart defaultStart;

    // idk the CAN ids
    private static int kIntakeMotorCanId;
    private static int kPivotMotorCanId;

    /* // PID values of the motor pivoting the intake
    // Need to find values - use zero as filler
    private static final double kPivotMotorP = 0.0;
    private static final double kPivotMotorI = 0.0;
    private static final double kPivotMotorD = 0.0;

    // i thought a PID controller might be useful, but i don't know where to use it
    private final PIDController mPivotPID = new PIDController(kPivotMotorP, kPivotMotorI, kPivotMotorD); */

    // make new pivot encoder, which is used to find angle of the pivot motor
    private final static DutyCycleEncoder mPivotEncoder = new DutyCycleEncoder(IntakeConstants.kPivotEncoderId);

    // make new motors (NEO 550) to control the intake rollers and the intake as a whole
    private static CANSparkMax intakeMotor1 = new CANSparkMax(kIntakeMotorCanId, MotorType.kBrushless);
    private CANSparkMax pivotMotor1 = new CANSparkMax(kPivotMotorCanId, MotorType.kBrushless);

    // maintain that there is one instance of the subsystem class
    public static synchronized IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    private IntakeSubsystem() {
        //
        intakeMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor1.setSmartCurrentLimit(40, 40);

        //
        pivotMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotMotor1.setSmartCurrentLimit(10, 10);

        // make new Defaultstart
        DefaultStart defaultStart = new DefaultStart();
    }

    private static class DefaultStart {
        // default pivot target is stow
        Double pivotTarget = IntakeConstants.kPivotAngleStow;

        // default intake is not moving
        Double intakeState = 0.0;
    }

    // make intake spin based on intake state
    public void setIntakeMotorEffort() {
        intakeMotor1.set(defaultStart.intakeState);
    }

    public double getPivotAngleDegrees() {
        // find current pivot angle
        double positionValue = mPivotEncoder.getAbsolutePosition() - IntakeConstants.kPivotEncoderOffset;
        // return current pivot angle in degrees
        return Units.rotationsToDegrees(positionValue);
    }

    public void pivotGoToGround() {
        // set pivot target to ground
        defaultStart.pivotTarget = IntakeConstants.kPivotAngleGround;
    }

    public void pivotGoToStow() {
        // set pivot target to stow
        defaultStart.pivotTarget = IntakeConstants.kPivotAngleStow;
    }

    public void intakeEffort() {
        // set intake target effort to intake
        defaultStart.intakeState = IntakeConstants.kIntakeEffort;
    }

    public void ejectEffort() {
        // set intake target effort to eject
        defaultStart.intakeState = IntakeConstants.kEjectEffort;
    }

    public void feedLauncherEffort() {
        // set intake target effort to feed
        defaultStart.intakeState = IntakeConstants.kFeedLauncherEffort;
    }

    // need to use for something, but seemed useful to have --- taken out because unnecessary at the moment
    /* 
    private boolean isPivotAtTarget() {
        // if the difference between the current angle and target angle of the pivot is less than five, pivot is at target
        return Math.abs(getPivotAngleDegrees() - defaultStart.pivotTarget) < some number;
    }*/

    @Override
    public void periodic() {

    }
}