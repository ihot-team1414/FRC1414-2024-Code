package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;

public class AmpSubsystem extends SubsystemBase {
    private static AmpSubsystem instance;

    private final CANSparkMax ampMotor1;
    private final CANSparkMax ampMotor2;

    public AmpSubsystem() {
        /*
         * Initialize CAN IDs.
         */
        ampMotor1 = new CANSparkMax(AmpConstants.kAmpRightCanId, MotorType.kBrushless);
        ampMotor2 = new CANSparkMax(AmpConstants.kAmpLeftCanId, MotorType.kBrushless);

        ampMotor1.setSmartCurrentLimit(5);
        ampMotor2.setSmartCurrentLimit(5);

        /*
         * Configure motion magic constants.
         */
        ampMotor1.getPIDController().setP(AmpConstants.kAmpP);
        ampMotor1.getPIDController().setI(AmpConstants.kAmpI);
        ampMotor1.getPIDController().setD(AmpConstants.kAmpD);

        /*
         * Configure motor neutral modes.
         */
        ampMotor1.setIdleMode(IdleMode.kBrake);
        ampMotor2.setIdleMode(IdleMode.kBrake);

        /*
         * Set default controls.
         */
        ampMotor2.follow(ampMotor1, true);
    }

    public static synchronized AmpSubsystem getInstance() {
        if (instance == null) {
            instance = new AmpSubsystem();
        }
        return instance;
    }

    public void setPosition(double position) {
        ampMotor1.getPIDController().setReference(position, ControlType.kPosition);
    }

    public double getPosition() {
        return ampMotor1.getEncoder().getPosition();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(getPosition()
                - position) < AmpConstants.kErrorMargin;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Amp Position", ampMotor1.getEncoder().getPosition());
    }
}
