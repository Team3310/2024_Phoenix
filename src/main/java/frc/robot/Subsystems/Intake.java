package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Intake extends SubsystemBase{
    private static Intake instance;

    private final double gearRatio = 12.0/30.0;
    private final TalonFX frontIntake = new TalonFX(0, TunerConstants.kCANbusName);
    private final TalonFX topIntake = new TalonFX(3, TunerConstants.kCANbusName);
    private final TalonFX bottomIntake = new TalonFX(5, TunerConstants.kCANbusName);
    private final TalonFX beltMotor = new TalonFX(5, TunerConstants.kCANbusName); //TODO change id


    private DigitalInput indexerSensor = new DigitalInput(0);
    private DigitalInput upSensor = new DigitalInput(1);

    private VelocityVoltage control = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    private final double kF = 0.11;
    private final double kP = 0.15;
    private final double kI = 0.012; 
    private final double kD = 0.0; 


    public static Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }

    private Intake(){
        frontIntake.setInverted(false);
        topIntake.setInverted(true);
        bottomIntake.setInverted(false);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kF;

        frontIntake.getConfigurator().apply(config.Slot0);
        topIntake.getConfigurator().apply(config.Slot0);
        bottomIntake.getConfigurator().apply(config.Slot0);
    }

    public void setFrontIntakeRPM(double rpm){
        rpm/=gearRatio; //tube to motor
        rpm/=60.0; //rpm to rps
        // SmartDashboard.putNumber("rps at motor", rpm);
        frontIntake.setControl(control.withVelocity(rpm));
    }

    public void setBackIntakeRPM(double rpm){
        setBottomIntakeRPM(rpm);
        setTopIntakeRPM(rpm);
    }

    public void setBottomIntakeRPM(double rpm){
        rpm/=gearRatio; //tube to motor
        rpm/=60.0; //rpm to rps          
        bottomIntake.setControl(control.withVelocity(rpm));
    }

    public void setTopIntakeRPM(double rpm){
        rpm/=gearRatio; //tube to motor
        rpm/=60.0; //rpm to rps
        topIntake.setControl(control.withVelocity(rpm));
    }

    public void setBeltIntakeRPM(double rpm){
        rpm/=gearRatio; //tube to motor
        rpm/=60.0; //rpm to rps
        beltMotor.setControl(control.withVelocity(rpm));
    }

    public void stopIntake(){
        setBackIntakeRPM(0);
        setFrontIntakeRPM(0);
        setBeltIntakeRPM(0.0);
    }

    public boolean getUpSensor(){
        return upSensor.get();
    }

    public boolean getIndexerSensor(){
        return indexerSensor.get();
    }

    public double getFrontIntakeRPM(){
        return frontIntake.getVelocity().getValueAsDouble()*60*gearRatio;
    }

    public double getTopIntakeRPM(){
        return topIntake.getVelocity().getValueAsDouble()*60*gearRatio;
    }

    public double getBottomIntakeRPM(){
        return bottomIntake.getVelocity().getValueAsDouble()*60*gearRatio;
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Front Intake RPM", getFrontIntakeRPM());
        // SmartDashboard.putNumber("Top Intake RPM", getTopIntakeRPM());
        // SmartDashboard.putNumber("Bottom Intake RPM", getBottomIntakeRPM());
    }
}
