package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Swerve.TunerConstants;

public class Climber extends SubsystemBase {
    private static Climber instance;

    private final TalonFX climberRight = new TalonFX(Constants.CLIMBER_LEFT_ID, TunerConstants.kSecondaryCANbusName);
    private final TalonFX climberLeft = new TalonFX(Constants.CLIMBER_RIGHT_ID, TunerConstants.kSecondaryCANbusName);

    private final DigitalInput chainSensor = new DigitalInput(Constants.CHAIN_SENSOR_PORT);

    private MotionMagicDutyCycle control = new MotionMagicDutyCycle(0);
    private DutyCycleOut speedControl = new DutyCycleOut(0);

    public enum ClimbControlMode{
        MANUAL, MOTION_MAGIC
    }

    private ClimbControlMode controlModeLeft = ClimbControlMode.MANUAL;
    private ClimbControlMode controlModeRight = ClimbControlMode.MANUAL;
    private boolean isZeroing = false;

    private double manualRightSpeed = 0;
    private double manualLeftSpeed = 0;

    // private SimpleDifferentialMechanism mechanism = new
    // SimpleDifferentialMechanism(climberMaster, climberSlave, true);

    private final double kF = 0.0;
    private final double kP = 2.0;
    private final double kI = 0.0;
    private final double kD = 0.0;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    private Climber() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kF;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getInchesToRotations(Constants.CLIMBER_MAX_INCHES);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getInchesToRotations(Constants.CLIMBER_MIN_INCHES);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        outputConfigs.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = getInchesToRotations(18.0); // inches per second
        config.MotionMagic.MotionMagicAcceleration = getInchesToRotations(18.0);

        climberRight.getConfigurator().apply(config);
        climberRight.getConfigurator().apply(outputConfigs);
        climberRight.setInverted(false);
        climberLeft.getConfigurator().apply(config);
        climberLeft.getConfigurator().apply(outputConfigs);
        climberLeft.setInverted(true);

        control.EnableFOC = true;

        // mechanism.setStaticBrake();
        // mechanism.applyConfigs();

        // climberSlave.setControl(new Follower(climberMaster.getDeviceID(), true));
    }

    public void setControlModeLeft(ClimbControlMode controlMode){
        this.controlModeLeft = controlMode;
    }

    public ClimbControlMode getControlModeLeft(){
        return controlModeLeft;
    }

    public void setControlModeRight(ClimbControlMode controlMode){
        this.controlModeRight = controlMode;
    }

    public ClimbControlMode getControlModeRight(){
        return controlModeRight;
    }

    public void setZeroing(boolean zeroing) {
        isZeroing = zeroing;
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        climberRight.setControl(speedControl.withOutput(rightSpeed));
        climberLeft.setControl(speedControl.withOutput(leftSpeed));
    }

    public void setLeftSpeed(double speed) {
        if(!isZeroing) {
            manualLeftSpeed = speed;

            controlModeLeft = ClimbControlMode.MANUAL;
            if (getLeftPositionInches() < Constants.CLIMBER_MIN_INCHES && speed < 0.0) {
                setHoldLeftClimber();;
            } else if (getLeftPositionInches() > Constants.CLIMBER_MAX_INCHES && speed > 0.0) {
                setHoldLeftClimber();;
            } else {
                climberLeft.setControl(speedControl.withOutput(speed));
            }
        }
    }

    public void setRightSpeed(double speed) {
        if(!isZeroing) {
            manualRightSpeed = speed;

            controlModeRight = ClimbControlMode.MANUAL;
            if (getRightPositionInches() < Constants.CLIMBER_MIN_INCHES && speed < 0.0) {
                setHoldRightClimber();
            } else if (getRightPositionInches() > Constants.CLIMBER_MAX_INCHES && speed > 0.0) {
                setHoldRightClimber();
            } else {
                climberRight.setControl(speedControl.withOutput(speed));
            }
        }
    }

    public void setPosition(double inches) {
        setLeftPositionInches(inches);
        setRightPositionInches(inches);
    }

    public void setLeftPositionInches(double inches) {
        controlModeLeft = ClimbControlMode.MOTION_MAGIC;
        climberLeft.setControl(control.withPosition(getInchesToRotations(inches)));
    }

    public void setRightPositionInches(double inches) {
        controlModeRight = ClimbControlMode.MOTION_MAGIC;
        climberRight.setControl(control.withPosition(getInchesToRotations(inches)));
    }

    public void setClimberZero(double inches) {
        climberRight.setPosition(getInchesToRotations(inches));
        climberLeft.setPosition(getInchesToRotations(inches));
    }

    private double getInchesToRotations(double inches) {
        if (inches > Constants.CLIMBER_MAX_INCHES) {
            inches = Constants.CLIMBER_MAX_INCHES;
        } else if (inches < Constants.CLIMBER_MIN_INCHES) {
            inches = Constants.CLIMBER_MIN_INCHES;
        }
        // SmartDashboard.putNumber("Commanded Rotations", inches * Constants.CLIMBER_GEAR_RATIO / (Math.PI * Constants.CLIMBER_PULLEY_DIAMETER));
        return inches * Constants.CLIMBER_GEAR_RATIO / (Math.PI * Constants.CLIMBER_PULLEY_DIAMETER);
    }

    private double getRotationsToInches(double rotations) {
        return rotations / Constants.CLIMBER_GEAR_RATIO * (Math.PI * Constants.CLIMBER_PULLEY_DIAMETER);
    }

    public double getRightPositionInches() {
        return getRotationsToInches(climberRight.getPosition().getValueAsDouble());
    }

    public double getLeftPositionInches() {
        return getRotationsToInches(climberLeft.getPosition().getValueAsDouble());
    }

    public double getRightCurrent() {
        return climberRight.getTorqueCurrent().getValueAsDouble();
    }

    public double getLeftCurrent() {
        return climberLeft.getTorqueCurrent().getValueAsDouble();
    }

    public synchronized void setHoldLeftClimber(){
        if(!isZeroing) {
            controlModeLeft = ClimbControlMode.MOTION_MAGIC;
            setLeftPositionInches(getLeftPositionInches());
        }
    }

    public synchronized void setHoldRightClimber(){
        if(!isZeroing) {
            controlModeRight = ClimbControlMode.MOTION_MAGIC;
            setRightPositionInches(getRightPositionInches());
        }
    }

    public boolean isChainDetected() {
        return chainSensor.get();
    }

    @Override
    public void periodic() {
        if (Constants.debug) {
            SmartDashboard.putNumber("Climber Left Inches", getLeftPositionInches());
            SmartDashboard.putNumber("Climber Right Inches", getRightPositionInches());
        }

        if(!isZeroing) {
            if (controlModeLeft == ClimbControlMode.MANUAL) {
                if (getLeftPositionInches() < Constants.CLIMBER_MIN_INCHES && manualLeftSpeed < 0.0) {
                    setHoldLeftClimber();
                } else if (getLeftPositionInches() > Constants.CLIMBER_MAX_INCHES && manualLeftSpeed > 0.0) {
                    setHoldLeftClimber();
                }
            }

            if (controlModeRight == ClimbControlMode.MANUAL) {
               if (getRightPositionInches() < Constants.CLIMBER_MIN_INCHES && manualRightSpeed < 0.0) {
                    setHoldRightClimber();
                } else if (getRightPositionInches() > Constants.CLIMBER_MAX_INCHES && manualRightSpeed > 0.0) {
                    setHoldRightClimber();
                }
            }
        }
    }
}
