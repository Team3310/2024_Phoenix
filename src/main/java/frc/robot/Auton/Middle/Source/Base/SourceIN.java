package frc.robot.Auton.Middle.Source.Base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Paths;

public class SourceIN extends AutonCommandBase{
    public SourceIN(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().SOURCE_IN);

        this.addCommands(
            new OneAuton(robotContainer),
            FollowToIntake(Paths.getInstance().SOURCE_IN),
            GoToShoot(robotContainer, Paths.getInstance().IN_SM)
        );
    }
}
