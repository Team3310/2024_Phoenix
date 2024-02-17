package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private static Climber instance;

    private final TalonFX climberMaster = new TalonFX(Constants.CLIMBER_MASTER_ID);
    private final TalonFX climberSlave = new TalonFX(Constants.CLIMBER_SLAVE_ID);

    private PositionVoltage control = new PositionVoltage(0);

    // private SimpleDifferentialMechanism mechanism = new SimpleDifferentialMechanism(climberMaster, climberSlave, true);

    private final double kF = 0.0;
    private final double kP = 5.0;
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

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.CLIMBER_MAX_INCHES;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.CLIMBER_MIN_INCHES;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        climberMaster.getConfigurator().apply(config);
        climberSlave.getConfigurator().apply(config);

        // mechanism.setStaticBrake();
        // mechanism.applyConfigs();

        climberSlave.setControl(new Follower(climberMaster.getDeviceID(), false));
    }

    public void setPosition(double inches){
        // mechanism.setControl()
        climberMaster.setControl(control.withPosition(getInchesToRotations(inches)));
    }

    public void setElevatorZero(double inches){
        climberMaster.setPosition(getInchesToRotations(inches));
        climberSlave.setPosition(getInchesToRotations(inches));
    }

    private double getInchesToRotations(double inches){
        if(inches>Constants.CLIMBER_MAX_INCHES){
            inches = Constants.CLIMBER_MAX_INCHES;
        }else if(inches<Constants.CLIMBER_MIN_INCHES){
            inches = Constants.CLIMBER_MIN_INCHES;
        }
        return inches*(Constants.CLIMBER_GEAR_RATIO/(Math.PI * Constants.CLIMBER_PULLY_DIAMTER));
    }

    private double getRotationsToInches(double rotations){
        return rotations/(Constants.CLIMBER_GEAR_RATIO*(Math.PI * Constants.CLIMBER_PULLY_DIAMTER));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Master Inches", getRotationsToInches(climberMaster.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Climber Slave Inches", getRotationsToInches(climberSlave.getPosition().getValueAsDouble()));
    }
}
