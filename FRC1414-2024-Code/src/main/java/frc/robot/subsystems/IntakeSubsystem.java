package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.utils.RobotState;
import frc.utils.RobotState.RobotConfiguration;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    private final TalonFX intakeMotor1;
    private final TalonFX intakeMotor2;
    private final TimeOfFlight intakeSensorTop;
    private final TimeOfFlight intakeSensorBottom;
    // private final TimeOfFlight intakeSensorSide;
    private Follower followerControl;
    private DutyCycleOut dutyCycleOutControl;
    private TalonFXConfiguration intakeMotorConfiguration;

    private Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
    private XboxController driver = new XboxController(OIConstants.kOperatorControllerPort);

    public IntakeSubsystem() {
        /*
         * Initialize CAN IDs.
         */

        intakeMotor1 = new TalonFX(IntakeConstants.kIntakeMotor1CanId);
        intakeMotor2 = new TalonFX(IntakeConstants.kIntakeMotor2CanId);
        intakeSensorTop = new TimeOfFlight(IntakeConstants.kIntakeSensorTopCandId);
        intakeSensorBottom = new TimeOfFlight(IntakeConstants.kIntakeSensorBottomCanId);

        /*
         * Configure motor current limit.
         */
        intakeMotorConfiguration = new TalonFXConfiguration();
        intakeMotorConfiguration.withSlot0(IntakeConstants.kIntakeConfiguration);
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 10;

        intakeMotor1.getConfigurator().apply(intakeMotorConfiguration);
        intakeMotor2.getConfigurator().apply(intakeMotorConfiguration);

        /*
         * Configure motor neutral modes.
         */
        setDesiredNeutralMode();

        /*
         * Configure sensor.
         */
        intakeSensorTop.setRangingMode(RangingMode.Short, 24);
        intakeSensorBottom.setRangingMode(RangingMode.Short, 24);

        /*
         * Set default controls.
         */
        dutyCycleOutControl = new DutyCycleOut(0, true, false, false, false);
        followerControl = new Follower(intakeMotor1.getDeviceID(), true);
        intakeMotor2.setControl(followerControl);
    }

    public static synchronized IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    /*
     * Control Methods
     */
    public void setDesiredNeutralMode() {
        intakeMotor1.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setDutyCycle(double dutyCycle) {
        intakeMotor1.setControl(dutyCycleOutControl.withOutput(dutyCycle));
    }

    public void stop() {
        intakeMotor1.stopMotor();
    }

    /*
     * Sensor Methods
     */
    private boolean isLoaded() {
        double distanceTop = intakeSensorTop.getRange();
        double distanceBottom = intakeSensorBottom.getRange();
        return (intakeSensorTop.isRangeValid() && distanceTop < IntakeConstants.kIndexThresholdTop)
                || (intakeSensorBottom.isRangeValid() && distanceBottom < IntakeConstants.kIndexThresholdBottom);
    }

    public boolean isLoadedDebounced() {
        return debouncer.calculate(isLoaded());
    }

    public void rumble(){
        driver.setRumble(RumbleType.kLeftRumble, 1);
    }

    public void stopRumble(){
        driver.setRumble(RumbleType.kLeftRumble, 0);
    }

    @Override
    public void periodic() {
        RobotState.getInstance().setHasNote(isLoadedDebounced());
        SmartDashboard.putNumber("Top Range", intakeSensorTop.getRange());
        SmartDashboard.putNumber("Bottom Range", intakeSensorBottom.getRange());
        SmartDashboard.putBoolean("No debounce", isLoaded());
        SmartDashboard.putBoolean("Valid???", intakeSensorTop.isRangeValid());
        
        if(RobotState.getInstance().getRobotConfiguration() == RobotConfiguration.INTAKING){
            rumble();
        } else { stopRumble(); } 
    }
}
