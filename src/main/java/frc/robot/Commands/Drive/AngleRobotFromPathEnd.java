package frc.robot.Commands.Drive;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Camera.Targeting;

public class AngleRobotFromPathEnd extends Command{
    private Drivetrain drivetrain;
    private Pose2d pos;
    public AngleRobotFromPathEnd(PathPlannerPath path){
        this.drivetrain = TunerConstants.DriveTrain;
        this.pos = path.getPathPoses().get(path.getPathPoses().size()-1);
    }

    @Override
    public void initialize() {
        drivetrain.snapToAngleAuton(Targeting.getTargetAzElFromPoint(pos)[0]);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished(){
        return !drivetrain.isSnapping;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
