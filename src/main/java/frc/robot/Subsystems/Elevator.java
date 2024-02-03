package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private static Elevator instance;

    private final TalonFX elevatorMaster = new TalonFX(Constants.ELEVATOR_MASTER_ID);
    private final TalonFX elevatorSlave = new TalonFX(Constants.ELEVATOR_SLAVE_ID);

    private PositionVoltage control = new PositionVoltage(0);

    private final double kF = 0.0;
    private final double kP = 5.0;
    private final double kI = 0.0; 
    private final double kD = 0.0; 


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

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ELEVATOR_MAX_INCHES;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ELEVATOR_MIN_INCHES;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        elevatorMaster.getConfigurator().apply(config.Slot0);

        elevatorSlave.setControl(new Follower(elevatorMaster.getDeviceID(), false));
    }

    public void setPosition(double inches){
        elevatorMaster.setControl(control.withPosition(getInchesToRotations(inches)));
    }

    public void setElevatorZero(double inches){
        elevatorMaster.setPosition(getInchesToRotations(inches));
        elevatorSlave.setPosition(getInchesToRotations(inches));
    }

    private double getInchesToRotations(double inches){
        if(inches>Constants.ELEVATOR_MAX_INCHES){
            return Constants.ELEVATOR_MAX_INCHES;
        }else if(inches<Constants.ELEVATOR_MIN_INCHES){
            return Constants.ELEVATOR_MIN_INCHES;
        }
        return Constants.ELEVATOR_GEAR_RATIO/(Math.PI * Constants.ELEVATOR_PULLY_DIAMTER);
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Front Intake RPM", getFrontIntakeRPM());
        // SmartDashboard.putNumber("Top Intake RPM", getTopIntakeRPM());
        // SmartDashboard.putNumber("Bottom Intake RPM", getBottomIntakeRPM());
    }
}
