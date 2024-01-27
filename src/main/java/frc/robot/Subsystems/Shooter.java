package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Shooter extends SubsystemBase{
    private static Shooter instance;

    private final double shooterRpmToMotorRPS = Constants.SHOOTER_GEAR_RATIO*60;
    private final double hoodDegreesToMotorRevs = Constants.HOOD_GEAR_RATIO*360;

    private final TalonFX shooter = new TalonFX(Constants.SHOOTER_MASTER_ID);
    private final TalonFX shooterSlave = new TalonFX(Constants.SHOOTER_SLAVE_ID);
    private final TalonFX hood = new TalonFX(Constants.HOOD_ID);

    private PositionVoltage hoodControl = new PositionVoltage(0);
    private VelocityVoltage shooterControl = new VelocityVoltage(0);


    private final double kF = 0.0;
    private final double kP = 5.0;
    private final double kI = 0.0; 
    private final double kD = 0.0; 


    public static Shooter getInstance(){
        if(instance == null){
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kF;

        shooter.getConfigurator().apply(config.Slot0);
        shooterSlave.getConfigurator().apply(config.Slot0);

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.HOOD_MAX_DEGREES;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.HOOD_MIN_DEGREES;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        hood.getConfigurator().apply(config.Slot0);

        //TODO check inverts

        shooterSlave.setControl(new Follower(shooter.getDeviceID(), false));
    }

    public void setHoodAngle(double degrees){
        hood.setControl(hoodControl.withPosition(getHoodDegreesToRevs(degrees)));
    }

    private double getHoodDegreesToRevs(double degrees){
        if(degrees>Constants.HOOD_MAX_DEGREES){
            return Constants.HOOD_MAX_DEGREES;
        }else if(degrees<Constants.HOOD_MIN_DEGREES){
            return Constants.HOOD_MIN_DEGREES;
        }
        return degrees/hoodDegreesToMotorRevs;
    }

    public void setShooterRpm(double rpm){
        shooter.setControl(shooterControl.withVelocity(getShooterRPMtoRPS(rpm)));
    }

    private double getShooterRPMtoRPS(double rpm){
        return rpm/shooterRpmToMotorRPS;
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Front Intake RPM", getFrontIntakeRPM());
        // SmartDashboard.putNumber("Top Intake RPM", getTopIntakeRPM());
        // SmartDashboard.putNumber("Bottom Intake RPM", getBottomIntakeRPM());
    }
}
