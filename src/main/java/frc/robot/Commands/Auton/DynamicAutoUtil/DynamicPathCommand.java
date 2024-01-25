package frc.robot.Commands.Auton.DynamicAutoUtil;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class DynamicPathCommand extends Command{
    private Drivetrain drivetrain;
    private PathPlannerPath path1;
    private PathPlannerPath path2;
    private boolean resetStartPose;
    private final DecisionPoint decisionPoint;
    private boolean changed;

    public DynamicPathCommand(Drivetrain drivetrain, PathPlannerPath path1, PathPlannerPath path2, boolean resetStartPose, DecisionPoint decisionPoint){
        this.drivetrain = drivetrain;
        this.path1 = path1;
        this.path2 = path2;
        this.resetStartPose = resetStartPose;
        this.decisionPoint = decisionPoint;
        changed = false;
    }

    @Override
    public void initialize() {
        drivetrain.setPath(path1, resetStartPose);
    }

    @Override
    public void execute() {
        if(!drivetrain.hasTarget() && !changed){
            switch (decisionPoint) {
                case PERCENTAGE:
                    if(drivetrain.getPose().getX()<path1.getPoint((int)(path1.getAllPathPoints().size()*decisionPoint.getEndPoint())).position.getX() 
                    && drivetrain.getPose().getX()>path1.getPoint((int)(path1.getAllPathPoints().size()*decisionPoint.getStartPoint())).position.getX())
                        {
                            drivetrain.setPath(path2, false);
                            changed = true;
                        }
                    break;
                case TIME:
                    if(drivetrain.getPathTime()>decisionPoint.getStartPoint() && drivetrain.getPathTime()<decisionPoint.getEndPoint())
                        {
                            drivetrain.setPath(path2, false);
                            changed = true;
                        }
                default:
                    break;
            }
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

    public enum DecisionPoint{
        PERCENTAGE,
        TIME,
        ;

        private double startPoint;
        private double endPoint;

        public double getStartPoint(){
            return startPoint;
        }

        public double getEndPoint(){
            return endPoint;
        }

        public DecisionPoint setStartPoint(double start){
            this.startPoint = start;
            return this;
        }

        public DecisionPoint setEndPoint(double end){
            this.endPoint = end;
            return this;
        }
    }
}
