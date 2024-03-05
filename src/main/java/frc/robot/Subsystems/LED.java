package frc.robot.Subsystems;

// import com.ctre.phoenix.led.StrobeAnimation;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.Animation;
// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Swerve.TunerConstants;

public class LED extends SubsystemBase{
    // private CANdle candle;
    // private StrobeAnimation blink;

    private boolean animate = false;
    private boolean prevAnimate = false;
    private Color prevColor = new Color();
    // private Animation prevAnimation;
    private Color currColor = new Color();
    // private Animation currAnimation;

    private static LED instance;

    public static LED getInstance(){
        if(instance == null){
            instance = new LED();
        }
        return instance;
    }

    // private LED(){
    //     candle = new CANdle(0, "rio");

    //     CANdleConfiguration config = new CANdleConfiguration();

    //     config.brightnessScalar = 0.5;
    // //    config.stripType = LEDStripType.RGB;

    //     candle.configAllSettings(config);

    //     blink = new StrobeAnimation(0, 0, 0);
    //     blink.setSpeed(0.2);
    //     blink.setNumLed(8);
    //     blink.setLedOffset(0);
    // }

    public void setBlink(Color color){
    //     prevAnimation = currAnimation;
    //     prevAnimate = animate;
    //     prevColor = currColor;
        
    //     blink.setR((int)(color.red*255));
    //     blink.setG((int)(color.green*255));
    //     blink.setB((int)(color.blue*255));
    //     animate = true;
    //     currAnimation = blink;
    //     candle.animate(blink,0);
    //     // candle.setLEDs((int)color.red*255, (int)color.green*255, (int)color.blue*255);
    }

    public void setSolid(Color color){
    //     prevAnimation = currAnimation;
    //     prevAnimate = animate;
    //     prevColor = currColor;

    //     prevColor = currColor;
    //     candle.clearAnimation(0);
    //     candle.setLEDs((int)(color.red*255), (int)(color.green*255), (int)(color.blue*255));
    //     currColor = color;
    //     animate = false;
    }

    public void setOff(){
    //     candle.clearAnimation(0);
    //     candle.setLEDs(0,0,0);
    //     currColor = new Color(0, 0, 0);
    //     prevColor = currColor;
    //     animate=false;
    //     prevAnimate = false;
    }

    public void setPrevState(){
    //     if(prevAnimate){
    //         candle.animate(prevAnimation,0);
    //     }else{
    //         candle.clearAnimation(0);
    //         setSolid(prevColor);
    //     }
    }
}
