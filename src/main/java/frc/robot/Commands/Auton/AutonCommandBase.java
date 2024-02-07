package frc.robot.Commands.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.SideChooser.SideMode;
import frc.robot.util.SpotChooser.SpotMode;

public class AutonCommandBase extends SequentialCommandGroup {
    protected RobotContainer robotContainer;

    protected AutonCommandBase(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
    }

    protected Command follow(String pathName) {
        return new FollowPathCommand(getPath(pathName));
    }

    protected void resetRobotPose(PathPlannerPath path) {
        Pose2d start = path.getStartingDifferentialPose();
        SmartDashboard.putString("bot starting pose", start.toString());
        
        TunerConstants.DriveTrain.seedFieldRelative(path.getStartingDifferentialPose()); 

        SmartDashboard.putString("robot pose after reset", TunerConstants.DriveTrain.getPose().toString());       
    }

    protected boolean getFlip(){
        return robotContainer.getSide() == SideMode.RED;
    }

    protected PathPlannerPath getPath(String pathName){
        pathName+=robotContainer.getSide().toString();
        SmartDashboard.putString("pathName", pathName);
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return path;
    }

    protected Double getPathTime(String pathName){
        return getPath(pathName).getTrajectory(new ChassisSpeeds(0, 0, 0), Rotation2d.fromDegrees(0)).getTotalTimeSeconds();
    }
}
