package frc.robot.Subsystems;

import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.TunerConstants;

public class LED extends SubsystemBase{
    private CANdle candle;
    private StrobeAnimation blink;

    private static LED instance;

    public static LED getInstance(){
        if(instance == null){
            instance = new LED();
        }
        return instance;
    }

    private LED(){
        candle = new CANdle(0, TunerConstants.kSecondaryCANbusName);

        CANdleConfiguration config = new CANdleConfiguration();

        config.brightnessScalar = 0.75;
        // config.stripType = LEDStripType.RGB;

        candle.configAllSettings(config);

        blink = new StrobeAnimation(0, 0, 0);
    }

    public void setBlink(Color color){
        blink = new StrobeAnimation((int)color.red, (int)color.green, (int)color.blue);
        blink.setSpeed(50);
        candle.animate(blink);
    }

    public void setSolid(Color color){
        candle.setLEDs((int)color.red, (int)color.green, (int)color.blue);
    }

    public void setOff(){
        candle.setLEDs(0,0,0);
    }
}
