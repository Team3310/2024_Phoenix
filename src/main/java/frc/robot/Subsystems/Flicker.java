package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flicker extends SubsystemBase{
    private static Flicker instance;

    private final TalonFX motor = new TalonFX(Constants.AMP_MOTOR_ID);

    private VelocityDutyCycle control = new VelocityDutyCycle(0);

    private final double kF = 0.0;
    private final double kP = 1.0;
    private final double kI = 0.0; 
    private final double kD = 0.0; 


    public static Flicker getInstance(){
        if(instance == null){
            instance = new Flicker();
        }
        return instance;
    }

    private Flicker(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kF;

        // config.CurrentLimits.StatorCurrentLimit = 10;
        config.CurrentLimits.StatorCurrentLimitEnable = false; 

        motor.getConfigurator().apply(config.Slot0);

        motor.setInverted(true);
    }

    public void setRPM(double rpm){
        motor.setControl(new DutyCycleOut(rpm));
    }

    private double getRollerToMotorRPM(double rpm){
        return (rpm/60.0) * Constants.AMP_GEAR_RATIO;
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Front Intake RPM", getFrontIntakeRPM());
        // SmartDashboard.putNumber("Top Intake RPM", getTopIntakeRPM());
        // SmartDashboard.putNumber("Bottom Intake RPM", getBottomIntakeRPM());
    }
}
