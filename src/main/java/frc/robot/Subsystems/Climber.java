package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private static Climber instance;

    private final TalonFX climberRight = new TalonFX(Constants.CLIMBER_LEFT_ID);
    private final TalonFX climberLeft = new TalonFX(Constants.CLIMBER_RIGHT_ID);

    private MotionMagicDutyCycle control = new MotionMagicDutyCycle(0);
    private DutyCycleOut speedControl = new DutyCycleOut(0);

    // private SimpleDifferentialMechanism mechanism = new SimpleDifferentialMechanism(climberMaster, climberSlave, true);

    private final double kF = 0.0;
    private final double kP = 2.0;
    private final double kI = 0.0; 
    private final double kD = 0.0; 

    public static Climber getInstance(){
        if(instance == null){
            instance = new Climber();
        }
        return instance;
    }

    private Climber(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kF;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getInchesToRotations(Constants.CLIMBER_MAX_INCHES);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getInchesToRotations(Constants.CLIMBER_MIN_INCHES);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = getInchesToRotations(9.0); //inches per second
        config.MotionMagic.MotionMagicAcceleration = getInchesToRotations(9.0);

        climberRight.getConfigurator().apply(config);
        climberRight.setInverted(false);
        climberLeft.getConfigurator().apply(config);
        climberLeft.setInverted(true);

        // mechanism.setStaticBrake();
        // mechanism.applyConfigs();

        // climberSlave.setControl(new Follower(climberMaster.getDeviceID(), true));
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        climberRight.setControl(speedControl.withOutput(rightSpeed));
        climberLeft.setControl(speedControl.withOutput(leftSpeed));
    }

    public void setPosition(double inches){
        // mechanism.setControl()
        climberRight.setControl(control.withPosition(getInchesToRotations(inches)));
        climberLeft.setControl(control.withPosition(getInchesToRotations(inches)));
        // climberSlave.setControl(control.withPosition(getInchesToRotations(inches)));
    }

    public void setClimberZero(double inches){
        climberRight.setPosition(getInchesToRotations(inches));
        climberLeft.setPosition(getInchesToRotations(inches));
    }

    private double getInchesToRotations(double inches){
        if(inches>Constants.CLIMBER_MAX_INCHES){
            inches = Constants.CLIMBER_MAX_INCHES;
        }else if(inches<Constants.CLIMBER_MIN_INCHES){
            inches = Constants.CLIMBER_MIN_INCHES;
        }
        SmartDashboard.putNumber("Commanded Rotations", inches*Constants.CLIMBER_GEAR_RATIO/(Math.PI * Constants.CLIMBER_PULLEY_DIAMETER));
        return inches*Constants.CLIMBER_GEAR_RATIO/(Math.PI * Constants.CLIMBER_PULLEY_DIAMETER);
    }

    private double getRotationsToInches(double rotations){
        return rotations/Constants.CLIMBER_GEAR_RATIO*(Math.PI * Constants.CLIMBER_PULLEY_DIAMETER);
    }

    public double getRightPositionInches() {
        return getRotationsToInches(climberRight.getPosition().getValueAsDouble());
    }

    public double getLeftPositionInches() {
        return getRotationsToInches(climberLeft.getPosition().getValueAsDouble());
    }

    public double getRightCurrent() {
        return climberRight.getStatorCurrent().getValue();
    }

    public double getLeftCurrent() {
        return climberLeft.getStatorCurrent().getValue();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Left Inches", getRotationsToInches(climberRight.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Climber Right Inches", getRotationsToInches(climberLeft.getPosition().getValueAsDouble()));

        SmartDashboard.putNumber("Climber Left rps", climberRight.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Climber Right rps", climberLeft.getVelocity().getValueAsDouble());
    }
}
