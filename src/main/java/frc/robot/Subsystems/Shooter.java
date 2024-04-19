package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Swerve.TunerConstants;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    // Motor Controllers
    private TalonFX shooterRightMain;
    private TalonFX shooterLeftMain;
    private TalonFX shooterKicker;

    /* Be able to switch which control request to use based on a button press */
    /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
    private final VelocityVoltage m_voltageVelocityRight = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VelocityVoltage m_voltageVelocityLeft = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VelocityVoltage m_voltageVelocityKicker = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    private DigitalInput sensor;

    private final double shooterRpmToMotorRPS = Constants.SHOOTER_GEAR_RATIO / 60;
    private final double kickerRpmToMotorRPS = Constants.KICKER_GEAR_RATIO / 60;

    private double leftOffset = 0.0;
    private double rightOffset = 0.0;

    private boolean hasNote = false;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        shooterRightMain = new TalonFX(Constants.SHOOTER_RIGHT_ID, TunerConstants.kSecondaryCANbusName);
        shooterLeftMain = new TalonFX(Constants.SHOOTER_LEFT_ID, TunerConstants.kSecondaryCANbusName);
        shooterKicker = new TalonFX(Constants.SHOOTER_KICKER_ID, TunerConstants.kSecondaryCANbusName);

        sensor = new DigitalInput(0);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.NeutralMode = NeutralModeValue.Coast;

        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        // Peak output of 40 amps
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        configs.CurrentLimits.StatorCurrentLimitEnable = false;

        configs.CurrentLimits.SupplyCurrentLimit = 40.0;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 60.0;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shooterRightMain.getConfigurator().apply(configs);
            status = shooterRightMain.getConfigurator().apply(outputConfigs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply right configs, error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = shooterLeftMain.getConfigurator().apply(configs);
            status = shooterLeftMain.getConfigurator().apply(outputConfigs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply left configs, error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = shooterKicker.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply kicker configs, error code: " + status.toString());
        }

        shooterKicker.setInverted(false);
        shooterLeftMain.setInverted(false);
        shooterRightMain.setInverted(true);

        m_voltageVelocityRight.EnableFOC = true;
        m_voltageVelocityLeft.EnableFOC = true;
        m_voltageVelocityKicker.EnableFOC = true;
    }

    public boolean isNoteLoaded() {
        return sensor.get();
    }

    public double getRightMainRPM() {
        return shooterRightMain.getVelocity().getValue() / shooterRpmToMotorRPS;
    }

    public void setRightMainOff() {
        shooterRightMain.setControl(new DutyCycleOut(0.0));
    }

    public void setRightMainRPM(double rpm) {
        if (Math.abs(rpm) < 1.0) {
            shooterRightMain.setControl(new DutyCycleOut(0.0));
            return;
        }

        rpm += rightOffset;

        shooterRightMain.setControl(m_voltageVelocityRight.withVelocity(rpm * shooterRpmToMotorRPS)
                .withAcceleration(rpm * shooterRpmToMotorRPS / 5.0));
    }

    public void setLeftMainOff() {
        shooterLeftMain.setControl(new DutyCycleOut(0.0));
    }

    public double getLeftMainRPM() {
        return shooterLeftMain.getVelocity().getValue() / shooterRpmToMotorRPS;
    }

    public void setLeftMainRPM(double rpm) {
        if (Math.abs(rpm) < 1.0) {
            shooterLeftMain.setControl(new DutyCycleOut(0.0));
            return;
        }

        rpm += leftOffset;

        shooterLeftMain.setControl(m_voltageVelocityLeft.withVelocity(rpm * shooterRpmToMotorRPS)
                .withAcceleration(rpm * shooterRpmToMotorRPS / 5.0));
    }

    public void setKickerOff() {
        shooterKicker.setControl(new DutyCycleOut(0.0));
    }

    public double getKickerRPM() {
        return shooterKicker.getVelocity().getValue() / shooterRpmToMotorRPS;
    }

    public void setKickerSpeed(double speed) {
        shooterKicker.setControl(new DutyCycleOut(speed));
    }

    public void setKickerRPM(double rpm) {
        if (Math.abs(rpm) < 1.0) {
            shooterKicker.setControl(new DutyCycleOut(0.0));
            return;
        }
        shooterKicker.setControl(m_voltageVelocityKicker.withVelocity(rpm * kickerRpmToMotorRPS));
    }

    public void adjustRPMLeftOffset(double amount){
        leftOffset += amount;
    }

    public void adjustRPMRightOffset(double amount){
        rightOffset += amount;
    }

    public void resetRPMOffset(){
        leftOffset = 0.0;
        rightOffset = 0.0;
    }

    public void setNoteIn(boolean hasNote) {
        this.hasNote = hasNote; 
    }

    public boolean hasNote() {
        return this.hasNote;
    }

    @Override
    public void periodic() {
        if (Constants.debug) {
            SmartDashboard.putNumber("Shooter Right RPM", getRightMainRPM());
            SmartDashboard.putNumber("Shooter Left RPM", getLeftMainRPM());
            SmartDashboard.putNumber("Kicker RPM", getKickerRPM());
            SmartDashboard.putBoolean("isNoteLoaded", isNoteLoaded());
            SmartDashboard.putNumber("Shooter RPM Left Offset", leftOffset);
            SmartDashboard.putNumber("Shooter RPM Right Offset", rightOffset);
        }

        SmartDashboard.putBoolean("Shooter has note", hasNote());
    }
}
