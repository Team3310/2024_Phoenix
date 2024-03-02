package frc.robot.Commands.Lift;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Camera.Targeting;

public class AimLiftFromPathEnd extends Command{
    private Lift lift;
    private Shooter shooter;
    private Pose2d pos;

    public AimLiftFromPathEnd(Pose2d pos){
        this.lift = Lift.getInstance();
        this.shooter = Shooter.getInstance();
        this.pos = pos;
        addRequirements(lift, shooter);
    }

    @Override
    public void initialize(){
        double[] values = new double[6];//Targeting.getTargetAzElFromPoint(pos);
        lift.setLiftAngle(values[1]);
        shooter.setLeftMainRPM(values[2]);
        shooter.setRightMainRPM(values[3]);
    }

    @Override
    public void execute(){ 
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){

    }
}