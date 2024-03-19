package frc.robot.Auton.DynamicAutoUtil;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class DynamicPathCommand extends Command{
    private Drivetrain drivetrain;
    private PathPlannerPath path1, path2;
    private final DecisionPoint decisionPoint;
    private DecisionPoint decisionPoint2;
    private boolean changed, end, resetStartPose;

    public DynamicPathCommand(Drivetrain drivetrain, PathPlannerPath path1, PathPlannerPath path2, boolean resetStartPose, DecisionPoint decisionPoint){
        this.drivetrain = drivetrain;
        this.path1 = path1;
        this.path2 = path2;
        this.resetStartPose = resetStartPose;
        this.decisionPoint2 = DecisionPoint.NULL;
        changed = false;
        end = false;
        if(RobotContainer.getInstance().getSideChooser().getSelected()==SideMode.RED){
            double start = decisionPoint.getEndPoint();
            decisionPoint.setEndPoint(decisionPoint.getStartPoint()).setStartPoint(start);
        }
        if(decisionPoint.getSupplier()==null){
            decisionPoint.setSupplier(()->(!drivetrain.hasTarget()));
        }
        this.decisionPoint = decisionPoint;
    }

    @Override
    public void initialize() {
        drivetrain.setPath(path1, resetStartPose);
    }

    @Override
    public void execute() {
        if(!changed){
            switch (decisionPoint) {
                case PERCENTAGE:
                    // SmartDashboard.putNumber("x lower", path1.getPoint((int)((path1.getAllPathPoints().size()-1)*decisionPoint.getStartPoint())).position.getX());
                    // SmartDashboard.putNumber("x higher", path1.getPoint((int)((path1.getAllPathPoints().size()-1)*decisionPoint.getEndPoint())).position.getX());

                    if(drivetrain.getPose().getX()<path1.getPoint((int)((path1.getAllPathPoints().size()-1)*decisionPoint.getEndPoint())).position.getX() 
                    && drivetrain.getPose().getX()>path1.getPoint((int)((path1.getAllPathPoints().size()-1)*decisionPoint.getStartPoint())).position.getX()
                    && decisionPoint.getSupplier().getAsBoolean())
                        {
                            drivetrain.setPath(path2, false);
                            changed = true;
                        }
                    break;
                case LIMELIGHT:
                    // SmartDashboard.putNumber("distance", path1.getPoint(path1.getAllPathPoints().size()-1).position.getDistance(drivetrain.getPose().getTranslation()));
                    if(path1.getPoint((int)((path1.getAllPathPoints().size()-1)*decisionPoint.getEndPoint())).position.getDistance(drivetrain.getPose().getTranslation())<decisionPoint.getEndPoint() 
                    && path1.getPoint(path1.getAllPathPoints().size()-1).position.getDistance(drivetrain.getPose().getTranslation())<Constants.MAX_NOTE_DISTANCE 
                    && decisionPoint.getSupplier().getAsBoolean())
                        {
                            end = true;
                        }
                    break;    
                case TIME:
                    if(drivetrain.getPathTime()>decisionPoint.getStartPoint() && drivetrain.getPathTime()<decisionPoint.getEndPoint() && decisionPoint.getSupplier().getAsBoolean())
                    {
                        drivetrain.setPath(path2, false);
                        changed = true;
                    }
                    break;
                case CUSTOM:
                    if(decisionPoint.getSupplier().getAsBoolean()){
                        drivetrain.setPath(path2, false);
                        changed = true;
                    }        
                    break;    
                default:
                    break;
            }
        }

        if(changed){
            switch (decisionPoint2) {
                case PERCENTAGE:
                    if(drivetrain.getPose().getX()<path2.getPoint((int)((path2.getAllPathPoints().size()-1)*decisionPoint2.getEndPoint())).position.getX() 
                    && drivetrain.getPose().getX()>path2.getPoint((int)((path2.getAllPathPoints().size()-1)*decisionPoint2.getStartPoint())).position.getX()
                    && decisionPoint2.getSupplier().getAsBoolean())
                        {
                            end = true;
                        }
                    break;
                case LIMELIGHT:
                    if(path2.getPoint((int)((path2.getAllPathPoints().size()-1)*decisionPoint2.getEndPoint())).position.getDistance(drivetrain.getPose().getTranslation())<decisionPoint2.getEndPoint() 
                    && path2.getPoint(path2.getAllPathPoints().size()-1).position.getDistance(drivetrain.getPose().getTranslation())<Constants.MAX_NOTE_DISTANCE 
                    && decisionPoint2.getSupplier().getAsBoolean())
                        {
                            end = true;
                        }
                    break;
                case TIME:
                    if(drivetrain.getPathTime()>decisionPoint2.getStartPoint() && drivetrain.getPathTime()<decisionPoint2.getEndPoint() && decisionPoint2.getSupplier().getAsBoolean())
                    {
                        end = true;
                    }
                    break;
                case CUSTOM:
                    if(decisionPoint2.getSupplier().getAsBoolean()){
                        end = true;
                    }        
                    break;
                default:
                    break;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return drivetrain.pathDone() || end;
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted){
            drivetrain.stopPath();
            System.out.println("path interrupted");
        }
    }

    public boolean getChanged(){
        return this.changed;
    }

    public DynamicPathCommand setSecondStop(DecisionPoint decisionPoint){
        this.decisionPoint2 = decisionPoint;
        return this;
    }

    public enum DecisionPoint{
        PERCENTAGE,
        LIMELIGHT,
        TIME,
        CUSTOM,
        NULL
        ;

        private double startPoint;
        private double endPoint;
        private BooleanSupplier supplier;

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

        public DecisionPoint setSupplier(BooleanSupplier supplier){
            this.supplier = supplier;
            return this;
        }

        public BooleanSupplier getSupplier(){
            return this.supplier;
        }
    }
}
