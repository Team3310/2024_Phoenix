package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase{
    private static Shooter instance;

    // Motor Controllers
    private TalonFX shooterTopRightMaster;
    // private TalonFX shooterBottomRightSlave;
    private TalonFX shooterTopLeftMaster;
    // private TalonFX shooterBottomLeftSlave;
    private TalonFX shooterKicker;

    /* Be able to switch which control request to use based on a button press */
    /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
    private final VelocityVoltage m_voltageVelocityRight = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VelocityVoltage m_voltageVelocityLeft = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VelocityVoltage m_voltageVelocityKicker = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    /* Start at velocity 0, no feed forward, use slot 1 */
    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

    private static final String canBusName = "rio";

    private DigitalInput sensor;

    private final double shooterRpmToMotorRPS = Constants.SHOOTER_GEAR_RATIO/60;

    public static Shooter getInstance(){
        if(instance == null){
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter(){
        TalonFXConfiguration configs = new TalonFXConfiguration();

        shooterTopRightMaster = new TalonFX(Constants.SHOOTER_RIGHT_MASTER_ID, canBusName);
        // shooterBottomRightSlave = new TalonFX(41, canBusName);
        shooterTopLeftMaster = new TalonFX(Constants.SHOOTER_LEFT_MASTER_ID, canBusName);
        // shooterBottomLeftSlave = new TalonFX(51, canBusName);
        shooterKicker = new TalonFX(Constants.SHOOTER_KICKER_ID, canBusName);

        sensor = new DigitalInput(0);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.NeutralMode = NeutralModeValue.Coast;

        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        
        /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
        configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

        // Peak output of 40 amps
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shooterTopRightMaster.getConfigurator().apply(configs);
            status = shooterTopRightMaster.getConfigurator().apply(outputConfigs);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Could not apply right configs, error code: " + status.toString());
        }
        // shooterBottomRightSlave.setControl(new Follower(shooterTopRightMaster.getDeviceID(), true));
 
        for (int i = 0; i < 5; ++i) {
            status = shooterTopLeftMaster.getConfigurator().apply(configs);
            status = shooterTopLeftMaster.getConfigurator().apply(outputConfigs);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Could not apply left configs, error code: " + status.toString());
        }
        // shooterBottomLeftSlave.setControl(new Follower(shooterTopLeftMaster.getDeviceID(), true));

        for (int i = 0; i < 5; ++i) {
            status = shooterKicker.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Could not apply kicker configs, error code: " + status.toString());
        }
    }

    public boolean isNoteLoaded() {
        return sensor.get();
    }

    public double getRightMainRPM() {
        return shooterTopRightMaster.getVelocity().getValue() / shooterRpmToMotorRPS;
    }

    public void setRightMainOff() {
        shooterTopRightMaster.setControl(new DutyCycleOut(0.0));
    }

    public void setRightMainRPM(double rpm) {
        shooterTopRightMaster.setControl(m_voltageVelocityRight.withVelocity(rpm*shooterRpmToMotorRPS));
//        shooterTopRightMaster.setControl(m_torqueVelocity.withVelocity(rpm/60.0).withFeedForward(1));
    }

    public void setLeftMainOff() {
        shooterTopLeftMaster.setControl(new DutyCycleOut(0.0));
    }

    public double getLeftMainRPM() {
        return shooterTopLeftMaster.getVelocity().getValue() / shooterRpmToMotorRPS;
    }

    public void setLeftMainRPM(double rpm) {
        shooterTopLeftMaster.setControl(m_voltageVelocityLeft.withVelocity(rpm*shooterRpmToMotorRPS));
//        shooterTopLeftMaster.setControl(m_torqueVelocity.withVelocity(rpm/60.0).withFeedForward(1));
    }

    public void setKickerOff() {
        shooterKicker.setControl(new DutyCycleOut(0.0));
    }

    public double getKickerRPM() {
        return shooterKicker.getVelocity().getValue() / shooterRpmToMotorRPS;
    }

    public void setKickerRPM(double rpm) {
        shooterKicker.setControl(m_voltageVelocityKicker.withVelocity(rpm*shooterRpmToMotorRPS));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Right RPM", getRightMainRPM());
        SmartDashboard.putNumber("Shooter Left RPM", getLeftMainRPM());
        SmartDashboard.putNumber("Kicker RPM", getKickerRPM());
        SmartDashboard.putBoolean("isNoteLoaded", isNoteLoaded());
    }
}
