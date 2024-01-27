package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Flicker extends SubsystemBase{
    private static Flicker instance;

    private final double gearRatio = 1.0/40.0;
    private final double maxPos = 5.0;//3.736816; //rotations
    private final double minPos = 0.0;//8.763184;
    private final TalonFX motor = new TalonFX(10);

    private PositionVoltage control = new PositionVoltage(0);

    private final double kF = 0.0;
    private final double kP = 5.0;
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

        config.CurrentLimits.StatorCurrentLimit = 10;
        config.CurrentLimits.StatorCurrentLimitEnable = true; 

        motor.getConfigurator().apply(config.Slot0);

        motor.setPosition(minPos);
        motor.setInverted(true);
    }

    public void setPosition(double percent){
        control.withPosition(percent*maxPos);
        SmartDashboard.putNumber("control pos", control.Position);
        motor.setControl(control);
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Front Intake RPM", getFrontIntakeRPM());
        // SmartDashboard.putNumber("Top Intake RPM", getTopIntakeRPM());
        // SmartDashboard.putNumber("Bottom Intake RPM", getBottomIntakeRPM());
    }
}
