package frc.robot.Commands.Auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Choosers.SideChooser.SideMode;
import frc.robot.util.Choosers.SpotChooser.SpotMode;

public class AutonCommandBase extends SequentialCommandGroup {
    protected RobotContainer robotContainer;

    protected AutonCommandBase(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
    }

    protected Command follow(PathPlannerPath path) {
        return new FollowPathCommand(path);
    }

    protected Command follow(String pathName) {
        return new FollowPathCommand(getPath(pathName));
    }

    protected void resetRobotPose(PathPlannerPath path) {
        path.getTrajectory(new ChassisSpeeds(0, 0, 0), path.getStartingDifferentialPose().getRotation()).getInitialDifferentialPose();
        Pose2d start = path.getPreviewStartingHolonomicPose();
        SmartDashboard.putString("bot starting pose", start.toString());
        
        TunerConstants.DriveTrain.seedFieldRelative(start); 

        SmartDashboard.putString("robot pose after reset", TunerConstants.DriveTrain.getPose().toString());       
    }

    protected boolean getFlip(){
        return robotContainer.getSide() == SideMode.RED;
    }

    protected PathPlannerPath getPath(String pathName){
        SmartDashboard.putString("pathName", pathName);
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        if(robotContainer.getSide() == SideMode.RED){
            path.preventFlipping = false;
            path.flipPath();
        }
        return path;
    }

    protected Double getPathTime(String pathName){
        return getPath(pathName).getTrajectory(new ChassisSpeeds(0, 0, 0), Rotation2d.fromDegrees(0)).getTotalTimeSeconds();
    }
}
