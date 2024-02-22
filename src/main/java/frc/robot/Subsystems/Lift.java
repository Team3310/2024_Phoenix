package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift extends SubsystemBase{
    private static Lift instance;

    // Motor Controllers
    private TalonFX liftMotor;
    private CANcoder canCoder;

    private StatusSignal<Double> liftPositionRevs;

    private StatusSignal<Boolean> f_fusedSensorOutOfSync;
    private StatusSignal<Boolean> sf_fusedSensorOutOfSync;

    private final MotionMagicVoltage liftControlMotionMagicVoltage = new MotionMagicVoltage(0);
    
    private static final String canBusName = "rio";

    private int printCount = 0;

    public static Lift getInstance(){
        if(instance == null){
            instance = new Lift();
        }
        return instance;
    }

    private Lift(){
        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        canConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canConfig.MagnetSensor.MagnetOffset = -0.250;

        canCoder = new CANcoder(Constants.LIFT_CANCODER_ID, canBusName);
        canCoder.getConfigurator().apply(canConfig);

        TalonFXConfiguration configs = new TalonFXConfiguration();
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
        configs.MotionMagic.MotionMagicAcceleration = 80.0;
        configs.MotionMagic.MotionMagicJerk = 100.0;

        configs.CurrentLimits.StatorCurrentLimit = 30;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        configs.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        configs.Feedback.SensorToMechanismRatio = 1.0;
        configs.Feedback.RotorToSensorRatio = Constants.LIFT_GEAR_RATIO;

        liftMotor = new TalonFX(Constants.LIFT_MOTOR_ID, canBusName);
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            status = liftMotor.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        liftMotor.setInverted(false);

        f_fusedSensorOutOfSync = liftMotor.getFault_FusedSensorOutOfSync();
        sf_fusedSensorOutOfSync = liftMotor.getStickyFault_FusedSensorOutOfSync();

        liftPositionRevs = liftMotor.getPosition();
    }

    public void setLiftAngle(double degrees){
        if(degrees>Constants.LIFT_MAX_DEGREES){
            degrees = Constants.LIFT_MAX_DEGREES;
        }else if(degrees<Constants.LIFT_MIN_DEGREES){
            degrees = Constants.LIFT_MIN_DEGREES;
        }
        liftMotor.setControl(liftControlMotionMagicVoltage.withPosition(getLiftDegreesToRevs(degrees)).withSlot(0));
    }

    private double getLiftDegreesToRevs(double degrees){
        return (degrees/360.0); 
    }

    private double getLiftDegreesFromRevs(double revs){
        return revs * 360.0;
    }

    public double getLiftDegrees(){
        liftPositionRevs.refresh(); 
        return getLiftDegreesFromRevs(liftPositionRevs.getValue());
    }

    @Override
    public void periodic() {
        if (printCount++ > 10) {
            printCount = 0;
            // If any faults happen, print them out. Sticky faults will always be present if live-fault occurs
            f_fusedSensorOutOfSync.refresh();
            sf_fusedSensorOutOfSync.refresh();
            boolean anyFault = sf_fusedSensorOutOfSync.getValue();
            if (anyFault) {
              System.out.println("A fault has occurred:");
              /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
              if (f_fusedSensorOutOfSync.getValue()) {
                System.out.println("Fused sensor out of sync live-faulted");
              } else if (sf_fusedSensorOutOfSync.getValue()) {
                System.out.println("Fused sensor out of sync sticky-faulted");
              }
            }      
            SmartDashboard.putNumber("Lift Angle Deg", getLiftDegrees());
        }
    }
}
