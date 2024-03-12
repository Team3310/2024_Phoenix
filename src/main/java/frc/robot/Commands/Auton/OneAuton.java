package frc.robot.Commands.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;

public class OneAuton extends AutonCommandBase{
    public OneAuton(RobotContainer robotContainer, PathPlannerPath path){
        super(robotContainer);

        resetRobotPose(path);

        this.addCommands(
            AimAndShoot(robotContainer)
        );
    }
}
