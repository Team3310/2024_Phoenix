package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends SubsystemBase{
    private static Hood instance;

    // Motor Controllers
    private TalonFX hood;

    private final MotionMagicDutyCycle hoodControl = new MotionMagicDutyCycle(0);
    
    private static final String canBusName = "Drivetrain";

    private final double hoodDegreesToMotorRevs = Constants.HOOD_GEAR_RATIO/360;

    public static Hood getInstance(){
        if(instance == null){
            instance = new Hood();
        }
        return instance;
    }

    private Hood(){
        TalonFXConfiguration configs = new TalonFXConfiguration();

        hood = new TalonFX(Constants.HOOD_ID, canBusName);

        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.HOOD_MAX_DEGREES;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.HOOD_MIN_DEGREES;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configs.Slot0.kP = 0.5;
        configs.Slot0.kI = 0.008;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kV = 0.045;

        configs.MotionMagic.MotionMagicAcceleration = 5.86;
        configs.MotionMagic.MotionMagicCruiseVelocity = 2.93;
        configs.MotionMagic.MotionMagicExpo_kA = 0.0;
        configs.MotionMagic.MotionMagicExpo_kV = 0.0;
        configs.MotionMagic.MotionMagicJerk = 0.0;

        hood.getConfigurator().apply(configs.Slot0);
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

    private double getHoodDegreesFromRevs(double revs){
        return revs / hoodDegreesToMotorRevs;
    }

    public double getHoodDegrees(){
        return getHoodDegreesFromRevs(hood.getPosition().getValue());
    }

    public void setHoodZero(double angle){
        hood.setPosition(getHoodDegreesToRevs(angle));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Angle", getHoodDegrees());
    }
}
