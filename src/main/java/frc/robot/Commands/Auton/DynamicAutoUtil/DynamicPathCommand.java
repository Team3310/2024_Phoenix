package frc.robot.Commands.Auton.DynamicAutoUtil;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Commands.IntakeIn;
import frc.robot.Subsystems.Drivetrain;

public class DynamicPathCommand extends Command{
    private Drivetrain drivetrain;
    private PathPlannerPath path1;
    private PathPlannerPath path2;
    private boolean resetStartPose;
    private double startDecisionPoint;
    private double endDecisionPoint;
    private boolean changed;

    /**
     * 
     * @param drivetrain
     * @param path1
     * @param path2
     * @param resetStartPose
     * @param decisionPoint - the percent of path completed before changing path
     */
    public DynamicPathCommand(Drivetrain drivetrain, PathPlannerPath path1, PathPlannerPath path2, boolean resetStartPose, double startDecisionPoint, double endDecisionPoint){
        this.drivetrain = drivetrain;
        this.path1 = path1;
        this.path2 = path2;
        this.resetStartPose = resetStartPose;
        this.startDecisionPoint = startDecisionPoint;
        this.endDecisionPoint = endDecisionPoint;
        changed = false;
    }

    @Override
    public void initialize() {
        drivetrain.setPath(path1, resetStartPose);
    }

    @Override
    public void execute() {
        if(!changed 
            && drivetrain.getPose().getX()<path1.getPoint((int)(path1.getAllPathPoints().size()*endDecisionPoint)).position.getX() 
            && drivetrain.getPose().getX()>path1.getPoint((int)(path1.getAllPathPoints().size()*startDecisionPoint)).position.getX() 
            && !drivetrain.hasTarget())
        {
            drivetrain.setPath(path2, false);
            changed = true;
        }
    }

    @Override
    public boolean isFinished(){
        SmartDashboard.putBoolean("changed", changed);
        return drivetrain.pathDone();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    public boolean getChanged(){
        return this.changed;
    }
}
