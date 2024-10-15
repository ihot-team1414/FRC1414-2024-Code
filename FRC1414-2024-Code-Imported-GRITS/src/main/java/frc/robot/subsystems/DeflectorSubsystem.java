package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeflectorConstants;

public class DeflectorSubsystem extends SubsystemBase {
    private static DeflectorSubsystem instance;

    private final CANSparkMax deflectorMotor1;
    private final CANSparkMax deflectorMotor2;

    public DeflectorSubsystem() {
        deflectorMotor1 = new CANSparkMax(DeflectorConstants.kDeflectorMotor1CanId, MotorType.kBrushless);
        deflectorMotor2 = new CANSparkMax(DeflectorConstants.kDeflectorMotor2CanId, MotorType.kBrushless);

        deflectorMotor1.setSmartCurrentLimit(DeflectorConstants.kDeflectorCurrentLimit);
        deflectorMotor2.setSmartCurrentLimit(DeflectorConstants.kDeflectorCurrentLimit);

        deflectorMotor1.getPIDController().setP(DeflectorConstants.kDeflectorP);
        deflectorMotor1.getPIDController().setI(DeflectorConstants.kDeflectorI);
        deflectorMotor1.getPIDController().setD(DeflectorConstants.kDeflectorD);

        deflectorMotor1.setIdleMode(IdleMode.kBrake);
        deflectorMotor2.setIdleMode(IdleMode.kBrake);

        deflectorMotor2.follow(deflectorMotor1, true);
    }

    public static synchronized DeflectorSubsystem getInstance() {
        if (instance == null) {
            instance = new DeflectorSubsystem();
        }
        return instance;
    }

    public void setPosition(double position) {
        deflectorMotor1.getPIDController().setReference(position, ControlType.kPosition);
    }

    public double getPosition() {
        return deflectorMotor1.getEncoder().getPosition();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(getPosition() - position) < DeflectorConstants.kDeflectorTolerance;
    }

    // Commands

    public Command rotateToPosition(double position) {
        return this.run(() -> setPosition(position)).until(() -> isAtPositionSetpoint(position));
    }

    public Command stow() {
        return this.run(() -> setPosition(DeflectorConstants.kDeflectorStowPosition))
                .until(() -> isAtPositionSetpoint(DeflectorConstants.kDeflectorStowPosition));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Deflector Position", deflectorMotor1.getEncoder().getPosition());
    }
}
