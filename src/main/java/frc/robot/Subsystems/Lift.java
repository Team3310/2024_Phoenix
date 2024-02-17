package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift extends SubsystemBase{
    private static Lift instance;

    // Motor Controllers
    private TalonFX hood;

    private final MotionMagicVoltage hoodControl = new MotionMagicVoltage(0);
    
    private static final String canBusName = "rio";

    private final double hoodRevsToMotorRevs = Constants.LIFT_GEAR_RATIO;

    public static Lift getInstance(){
        if(instance == null){
            instance = new Lift();
        }
        return instance;
    }

    private Lift(){
        TalonFXConfiguration configs = new TalonFXConfiguration();

        hood = new TalonFX(Constants.LIFT_MOTOR_ID, canBusName);

        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.LIFT_MAX_DEGREES;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.LIFT_MIN_DEGREES;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        configs.Slot0.kP = 40;
        configs.Slot0.kI = 0;
        configs.Slot0.kD = 0.1;
        configs.Slot0.kV = 0.12;
        configs.Slot0.kA = 0.01;
        configs.Slot0.kS = 0.35;
        // configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // configs.Slot0.kG = 0.0;
        
        configs.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        configs.MotionMagic.MotionMagicAcceleration = 40.0;
        configs.MotionMagic.MotionMagicJerk = 100.0;

        configs.CurrentLimits.StatorCurrentLimit = 30;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        configs.Feedback.FeedbackRemoteSensorID = Constants.LIFT_CANCODER_ID;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            status = hood.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    public void setHoodAngle(double degrees){
        hood.setControl(hoodControl.withPosition(getHoodDegreesToRevs(degrees)).withSlot(0));
    }

    private double getHoodDegreesToRevs(double degrees){
        if(degrees>Constants.LIFT_MAX_DEGREES){
            degrees = Constants.LIFT_MAX_DEGREES;
        }else if(degrees<Constants.LIFT_MIN_DEGREES){
            degrees = Constants.LIFT_MIN_DEGREES;
        }
        return (degrees/360.0) * hoodRevsToMotorRevs;
    }

    private double getHoodDegreesFromRevs(double revs){
        return (revs / hoodRevsToMotorRevs) * 360.0;
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
