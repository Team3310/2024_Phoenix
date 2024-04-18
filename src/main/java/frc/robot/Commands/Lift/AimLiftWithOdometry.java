package frc.robot.Commands.Lift;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;
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
            // lift.setLiftAngle(drive.getOdoTargeting().getEl());
            // shooter.setLeftMainRPM(drive.getOdoTargeting().getLeftShooterSpeed());
            // shooter.setRightMainRPM(drive.getOdoTargeting().getRightShooterSpeed());
            if (drive.canSeeTargetTag()) {
                led.setSolid(new Color(0,255,0));
            }else{
                led.setSolid(new Color(255,0,255));
            }

            drive.getOdoTargeting().update();
            double liftAngle = drive.getOdoTargeting().getEl();
            double leftSpeed = drive.getOdoTargeting().getLeftShooterSpeed();
            double rightSpeed = drive.getOdoTargeting().getRightShooterSpeed();
            if(Constants.debug){
                SmartDashboard.putNumber("AimLift Angle", liftAngle);
                SmartDashboard.putNumber("AimLift Leftspeed", leftSpeed);
                SmartDashboard.putNumber("AimLift Rightspeed", rightSpeed);
            }
            lift.setLiftAngle(liftAngle);
            shooter.setLeftMainRPM(leftSpeed);
            shooter.setRightMainRPM(rightSpeed);

                // if(drive.snapComplete() && lift.isFinished()){
                // }else{
                //     led.setBlink(new Color(243, 204, 20));
                // }
            
            // else{
            //     drive.getOdoTargeting().update();  
            //     lift.setLiftAngle(drive.getOdoTargeting().getEl());
            //     shooter.setLeftMainRPM(drive.getOdoTargeting().getLeftShooterSpeed());
            //     shooter.setRightMainRPM(drive.getOdoTargeting().getRightShooterSpeed());
            // }
        } else if(targetSimple == TargetSimple.CORNERPASS) {
            // double passLiftAngle = Constants.kPassLiftAngleMap
            //         .getInterpolated(new InterpolatingDouble((drive.getOdoTargeting().getDistance_XY_average() / 0.0254) / 12.0)).value;
            // double passLeftShooterSpeed = Constants.kPassLeftShooterMap
            //         .getInterpolated(new InterpolatingDouble((drive.getOdoTargeting().getDistance_XY_average() / 0.0254) / 12.0)).value;
            // double passRightShooterSpeed = Constants.kPassRightShooterMap
            //         .getInterpolated(new InterpolatingDouble((drive.getOdoTargeting().getDistance_XY_average() / 0.0254) / 12.0)).value;
            double passLiftAngle = Constants.kCornerLiftAngleFixed;
            double passLeftShooterSpeed = Constants.kCornerLeftShooterFixed;
            double passRightShooterSpeed = Constants.kCornerRightShooterFixed;

            lift.setLiftAngle(passLiftAngle);
            shooter.setLeftMainRPM(passLeftShooterSpeed);
            shooter.setRightMainRPM(passRightShooterSpeed);
        } else if(targetSimple == TargetSimple.CENTERPASS) {
            // double centerLiftAngle = Constants.kCenterLiftAngleMap
            //         .getInterpolated(new InterpolatingDouble((drive.getOdoTargeting().getDistance_XY_average() / 0.0254) / 12.0)).value;
            // double centerLeftShooterSpeed = Constants.kCenterLeftShooterMap
            //         .getInterpolated(new InterpolatingDouble((drive.getOdoTargeting().getDistance_XY_average() / 0.0254) / 12.0)).value;
            // double centerRightShooterSpeed = Constants.kCenterRightShooterMap
            //         .getInterpolated(new InterpolatingDouble((drive.getOdoTargeting().getDistance_XY_average() / 0.0254) / 12.0)).value;
            double centerLiftAngle = Constants.kCenterLiftAngleFixed;
            double centerLeftShooterSpeed = Constants.kCenterLeftShooterFixed;
            double centerRightShooterSpeed = Constants.kCenterRightShooterFixed;

            lift.setLiftAngle(centerLiftAngle);
            shooter.setLeftMainRPM(centerLeftShooterSpeed);
            shooter.setRightMainRPM(centerRightShooterSpeed);
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