package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flicker extends SubsystemBase {
    private static Flicker instance;

    private final TalonFX motor = new TalonFX(Constants.AMP_MOTOR_ID, Constants.rioCANbusName);
    private final DigitalInput sensor = new DigitalInput(Constants.AMP_SENSOR_PORT);

    private VelocityVoltage control = new VelocityVoltage(0);

    private final double kP = 0.11;
    private final double kI = 0.5;
    private final double kD = 0.0001;
    private final double kV = 0.12;

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
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;

        // config.CurrentLimits.StatorCurrentLimit = 10;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        motor.getConfigurator().apply(config.Slot0);

        motor.setInverted(false);

        control.EnableFOC = true;
    }

    public void setRPM(double rpm) {
        if (Math.abs(rpm) < 1.0) {
            motor.setControl(new DutyCycleOut(0.0));
            return;
        }
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
        // SmartDashboard.putNumber("Front Intake RPM", getFrontIntakeRPM());
        // SmartDashboard.putNumber("Top Intake RPM", getTopIntakeRPM());
        // SmartDashboard.putNumber("Bottom Intake RPM", getBottomIntakeRPM());
        SmartDashboard.putBoolean("is note in amp", isNoteLoaded());
    }
}
