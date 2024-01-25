package frc.robot.Commands.Auton;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.SideChooser.SideMode;

public class AutonCommandBase extends SequentialCommandGroup {
    protected RobotContainer robotContainer;

    protected AutonCommandBase(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
    }

    protected Command follow(String pathName) {
        return new InstantCommand(()->robotContainer.getDrivetrain().setPath(getPath(pathName), false));
    }

    protected void resetRobotPose(String pathName) {
        this.addCommands(new InstantCommand(()->TunerConstants.DriveTrain.
            seedFieldRelative(
                getPath(pathName).getStartingDifferentialPose()
            )));
    }

    protected boolean getFlip(){
        return robotContainer.getSide() == SideMode.RED;
    }

    protected PathPlannerPath getPath(String pathName){
        pathName+=robotContainer.getSpot().getName();
        return getFlip()?PathPlannerPath.fromPathFile(pathName).flipPath():PathPlannerPath.fromPathFile(pathName);
    }

    protected Double getPathTime(String pathName){
        return getPath(pathName).getTrajectory(new ChassisSpeeds(0, 0, 0), Rotation2d.fromDegrees(0)).getTotalTimeSeconds();
    }
}
