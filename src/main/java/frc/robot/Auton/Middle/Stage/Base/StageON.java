package frc.robot.Auton.Middle.Stage.Base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Paths;

public class StageON extends AutonCommandBase{
    public StageON(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().SOURCE_ON);

        this.addCommands(
            new OneAuton(robotContainer),
            FollowToIntake(Paths.getInstance().SOURCE_ON),
            GoToShoot(robotContainer, Paths.getInstance().ON_SM)
        );
    }
}