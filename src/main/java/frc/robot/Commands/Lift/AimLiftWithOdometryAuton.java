package frc.robot.Commands.Lift;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.TunerConstants;

public class AimLiftWithOdometryAuton extends Command{
    private Lift lift;
    private Drivetrain drive;
    private Shooter shooter;

    public AimLiftWithOdometryAuton(){
        this.lift = Lift.getInstance();
        this.drive = TunerConstants.DriveTrain;
        this.shooter = Shooter.getInstance();

        addRequirements(lift);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){ 
        if(Constants.debug){
            SmartDashboard.putBoolean("has target", drive.hasTarget());
        }
        if (drive.hasTarget()) {
            // if(Constants.debug){
            //     SmartDashboard.putNumber("targeting dist", drive.getLimelightTargeting().getDistance_XY_average());
            // }
            // drive.getLimelightTargeting().update();
            // // double angle = dist!=0.0?drive.getLimelightTargeting().getEl():Constants.kLiftAngleMapComp.getInterpolated(new InterpolatingDouble(dist)).value;
            // lift.setLiftAngle(drive.getLimelightTargeting().getEl());
            // shooter.setLeftMainRPM(drive.getLimelightTargeting().getLeftShooterSpeed());
            // shooter.setRightMainRPM(drive.getLimelightTargeting().getRightShooterSpeed());

            LED.getInstance().setSolid(Color.kGreen);
        }else{
            // if(Constants.debug){
            //     SmartDashboard.putNumber("targeting odo dist", drive.getOdoTargeting().getDistance_XY_average());
            // }
            // drive.getOdoTargeting().update();  
            // lift.setLiftAngle(drive.getOdoTargeting().getEl());
            // shooter.setLeftMainRPM(drive.getOdoTargeting().getLeftShooterSpeed());
            // shooter.setRightMainRPM(drive.getOdoTargeting().getRightShooterSpeed());
            LED.getInstance().setSolid(Color.kHotPink);
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
    }

    @Override
    public boolean isFinished(){
        return lift.isFinished();
    }

    @Override
    public void end(boolean interrupted){

    }
}