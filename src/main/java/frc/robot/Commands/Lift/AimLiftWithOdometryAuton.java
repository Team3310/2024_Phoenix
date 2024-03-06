package frc.robot.Commands.Lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
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
        if (drive.hasTarget()) {
            // drive.getLimelightTargeting().update();
            lift.setLiftAngle(drive.getLimelightTargeting().getEl());
            shooter.setLeftMainRPM(drive.getLimelightTargeting().getLeftShooterSpeed());
            shooter.setRightMainRPM(drive.getLimelightTargeting().getRightShooterSpeed());
        }else{
            drive.getOdoTargeting().update();  
            lift.setLiftAngle(drive.getOdoTargeting().getEl());
            shooter.setLeftMainRPM(drive.getOdoTargeting().getLeftShooterSpeed());
            shooter.setRightMainRPM(drive.getOdoTargeting().getRightShooterSpeed());
        }
    }

    @Override
    public boolean isFinished(){
        return lift.isFinished();
    }

    @Override
    public void end(boolean interrupted){

    }
}