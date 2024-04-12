package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Swerve.TunerConstants;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    private final TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR_ID, TunerConstants.kSecondaryCANbusName);

    private MotionMagicDutyCycle mmDutyCycleControl = new MotionMagicDutyCycle(0);
    private DutyCycleOut speedControl = new DutyCycleOut(0);

    private final double kV = 0.01;
    private final double kP = 0.5;
    private final double kI = 0.001;
    private final double kD = 0.0003;
    private final double kG = 0.03;
    private final double kA = 0.001;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    private Elevator() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.Slot0.kG = kG;
        config.Slot0.kA = kA;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getInchesToRotations(Constants.ELEVATOR_MAX_INCHES);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getInchesToRotations(Constants.ELEVATOR_MIN_INCHES);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        outputConfigs.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = 93.0; // rotations per second
        config.MotionMagic.MotionMagicAcceleration = 201.0;

        elevatorMotor.getConfigurator().apply(config);
        elevatorMotor.getConfigurator().apply(outputConfigs);

        mmDutyCycleControl.EnableFOC = true;

        elevatorMotor.setInverted(true);
    }

    public void setSpeed(double speed) {
        elevatorMotor.setControl(speedControl.withOutput(speed));
    }

    public void setPosition(double inches) {
        elevatorMotor.setControl(mmDutyCycleControl.withPosition(getInchesToRotations(inches)));
    }

    public void setElevatorZero(double inches) {
        elevatorMotor.setPosition(getInchesToRotations(inches));
    }

    private double getInchesToRotations(double inches) {
        if (inches > Constants.ELEVATOR_MAX_INCHES) {
            inches = Constants.ELEVATOR_MAX_INCHES;
        } else if (inches < Constants.ELEVATOR_MIN_INCHES) {
            inches = Constants.ELEVATOR_MIN_INCHES;
        }
        return inches * Constants.ELEVATOR_GEAR_RATIO / (Math.PI * Constants.ELEVATOR_PULLEY_DIAMETER);
    }

    private double getRotationsToInches(double rotations) {
        return rotations / Constants.ELEVATOR_GEAR_RATIO * (Math.PI * Constants.ELEVATOR_PULLEY_DIAMETER);
    }

    public double getPositionInches() {
        return getRotationsToInches(elevatorMotor.getPosition().getValueAsDouble());
    }

    public double getCurrent() {
        return elevatorMotor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        if(Constants.debug){
            SmartDashboard.putNumber("Elevator Inches", getPositionInches());
        }
    }
}
