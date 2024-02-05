package frc.robot.Commands.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        return new InstantCommand(()->robotContainer.getDrivetrain().setPath(getPath(pathName), false));
    }

    protected void resetRobotPose(PathPlannerPath path) {
        this.addCommands(new InstantCommand(()->TunerConstants.DriveTrain.
            seedFieldRelative(
                path.getStartingDifferentialPose(),
                path.getStartingDifferentialPose().getRotation()
            )));
    }

    protected boolean getFlip(){
        return robotContainer.getSide() == SideMode.RED;
    }

    protected PathPlannerPath getPath(String pathName){
        pathName+=robotContainer.getSpot().getName();
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        if(robotContainer.getSide()==SideMode.RED){
            path.flipPath();
        }
        return path;
    }

    protected Double getPathTime(String pathName){
        return getPath(pathName).getTrajectory(new ChassisSpeeds(0, 0, 0), Rotation2d.fromDegrees(0)).getTotalTimeSeconds();
    }
}
