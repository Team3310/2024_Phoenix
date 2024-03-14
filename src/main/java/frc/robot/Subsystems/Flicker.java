package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Swerve.TunerConstants;

public class Flicker extends SubsystemBase {
    private static Flicker instance;

    private final TalonFX motor = new TalonFX(Constants.AMP_MOTOR_ID, TunerConstants.kSecondaryCANbusName);
    private final DigitalInput sensor = new DigitalInput(Constants.AMP_SENSOR_PORT);

    private VelocityDutyCycle control = new VelocityDutyCycle(0);
 
    private final double kP = 1;
    private final double kI = 0.01;

    public static Flicker getInstance() {
        if (instance == null) {
            instance = new Flicker();
        }
        return instance;
    }

    private Flicker() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
 
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        motor.setInverted(false);

        control.EnableFOC = true;
    }

    public void setRPM(double rpm) {
        if (Math.abs(rpm) < 1.0) {
            motor.setControl(new DutyCycleOut(0.0));
            return;
        }
        // motor.setControl(new DutyCycleOut((rpm < 0) ? -0.5 : 0.5));
        motor.setControl(control.withVelocity(getRollerToMotorRPM(rpm)));
    }

    private double getRollerToMotorRPM(double rpm) {
        return (rpm / 60.0) * Constants.AMP_GEAR_RATIO;
    }

    public boolean isNoteLoaded() {
        return sensor.get();
    }

    @Override
    public void periodic() {
        if (Constants.debug) {
            SmartDashboard.putBoolean("is note in amp", isNoteLoaded());
            SmartDashboard.putNumber("Flicker RPM", motor.getVelocity().getValueAsDouble());
        }
    }
}
