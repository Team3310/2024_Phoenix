package frc.robot.Commands.Lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;

public class AimLiftWithOdometry extends Command{
    private Lift lift;
    private Drivetrain drive;

    public AimLiftWithOdometry(){
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
        return drive.getDriveMode()==DriveMode.JOYSTICK;
    }

    @Override
    public void end(boolean interrupted){

    }
}