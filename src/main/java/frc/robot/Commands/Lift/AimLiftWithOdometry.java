package frc.robot.Commands.Lift;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Camera.Targeting;
import frc.robot.util.Camera.Targeting.TargetSimple;

public class AimLiftWithOdometry extends Command{
    private Lift lift;
    private Drivetrain drive;
    private Shooter shooter;
    private LED led;
    private boolean setBlink = false;

    public AimLiftWithOdometry(){
        this.lift = Lift.getInstance();
        this.drive = TunerConstants.DriveTrain;
        this.shooter = Shooter.getInstance();
        this.led = LED.getInstance();

        addRequirements(lift, led);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){ 
        TargetSimple targetSimple = Targeting.getTargetSimple();
        if(targetSimple == TargetSimple.SPEAKER){
            if (drive.canSeeTargetTag()) {
                // drive.getLimelightTargeting().update();
                lift.setLiftAngle(drive.getLimelightTargeting().getEl());
                shooter.setLeftMainRPM(drive.getLimelightTargeting().getLeftShooterSpeed());
                shooter.setRightMainRPM(drive.getLimelightTargeting().getRightShooterSpeed());

                // if(drive.snapComplete() && lift.isFinished()){
                    led.setSolid(new Color(0,255,0));
                // }else{
                //     led.setBlink(new Color(243, 204, 20));
                // }
            }else{
                led.setSolid(new Color(255,0,0));
            }
            // else{
            //     drive.getOdoTargeting().update();  
            //     lift.setLiftAngle(drive.getOdoTargeting().getEl());
            //     shooter.setLeftMainRPM(drive.getOdoTargeting().getLeftShooterSpeed());
            //     shooter.setRightMainRPM(drive.getOdoTargeting().getRightShooterSpeed());
            // }
        } else if((targetSimple == TargetSimple.CENTERPASS) || (targetSimple == TargetSimple.CORNERPASS)) {
            lift.setLiftAngle(42.0);
            shooter.setLeftMainRPM(3200);
            shooter.setRightMainRPM(1700);
        }
    }

    @Override
    public boolean isFinished(){
        return drive.getDriveMode()==DriveMode.JOYSTICK;
    }

    @Override
    public void end(boolean interrupted){
        // led.setPrevState();
    }
}