package frc.robot.Commands.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;

public class FollowPathCommand extends Command{
    Drivetrain drive;
    PathPlannerPath path;

    public FollowPathCommand(PathPlannerPath path){
        this.path = path;
        drive = TunerConstants.DriveTrain;
    }

    @Override
    public void initialize(){
        drive.setPath(path, false);
        drive.setDriveMode(DriveMode.AUTON);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return drive.pathDone();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
