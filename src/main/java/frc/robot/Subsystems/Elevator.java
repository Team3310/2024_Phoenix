package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private static Elevator instance;

    private final TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR_ID);

    private MotionMagicDutyCycle control = new MotionMagicDutyCycle(0);

    private final double kF = 0.0;
    private final double kP = 1.0;
    private final double kI = 0.001; 
    private final double kD = 0.0003; 
    private final double kG = 0.04;


    public static Elevator getInstance(){
        if(instance == null){
            instance = new Elevator();
        }
        return instance;
    }

    private Elevator(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kF;
        config.Slot0.kG = kG;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getInchesToRotations(Constants.ELEVATOR_MAX_INCHES);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getInchesToRotations(Constants.ELEVATOR_MIN_INCHES);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotionMagic.MotionMagicCruiseVelocity = 121.0; //rotations per second
        config.MotionMagic.MotionMagicAcceleration = 201.0;

        elevatorMotor.getConfigurator().apply(config);
    }

    public void setPosition(double inches){
        elevatorMotor.setControl(control.withPosition(getInchesToRotations(inches)));
    }

    public void setElevatorZero(double inches){
        elevatorMotor.setPosition(getInchesToRotations(inches));
    }

    private double getInchesToRotations(double inches){
        if(inches>Constants.ELEVATOR_MAX_INCHES){
            inches = Constants.ELEVATOR_MAX_INCHES;
        }else if(inches<Constants.ELEVATOR_MIN_INCHES){
            inches = Constants.ELEVATOR_MIN_INCHES;
        }
        return inches*Constants.ELEVATOR_GEAR_RATIO/(Math.PI * Constants.ELEVATOR_PULLY_DIAMTER);
    }

    private double getRotationsToInches(double rotations){
        return rotations/Constants.ELEVATOR_GEAR_RATIO*(Math.PI * Constants.ELEVATOR_PULLY_DIAMTER);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Inches", getRotationsToInches(elevatorMotor.getPosition().getValueAsDouble()));
    }
}
