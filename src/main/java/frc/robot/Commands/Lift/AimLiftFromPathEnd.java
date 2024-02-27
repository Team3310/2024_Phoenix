package frc.robot.Commands.Lift;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Camera.Targeting;

public class AimLiftFromPathEnd extends Command{
    private Lift lift;
    private Pose2d pos;

    public AimLiftFromPathEnd(PathPlannerPath path){
        this.lift = Lift.getInstance();
        this.pos = path.getPathPoses().get(path.getPathPoses().size()-1);
        addRequirements(lift);
    }

    @Override
    public void initialize(){
        lift.setLiftAngle(Targeting.getTargetAzElFromPoint(pos)[1]);
    }

    @Override
    public void execute(){ 
        
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}