package frc.robot.Commands.Lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.TunerConstants;

public class AimLiftWithOdometryAuton extends Command{
    private Lift lift;
    private Drivetrain drive;

    public AimLiftWithOdometryAuton(){
        this.lift = Lift.getInstance();
        this.drive = TunerConstants.DriveTrain;

        addRequirements(lift);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){ 
        if (drive.hasTarget()) {
            drive.getLimelightTargeting().update();
            lift.setLiftAngle(drive.getLimelightTargeting().getEl());
        }else{
            drive.getOdoTargeting().update();  
            lift.setLiftAngle(drive.getOdoTargeting().getEl());
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){

    }
}