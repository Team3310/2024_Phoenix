package frc.robot.Auton.Middle.Stage.Base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Paths;

public class StageIN extends AutonCommandBase{
    public StageIN(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().SOURCE_IN);

        this.addCommands(
            new OneAuton(robotContainer),
            FollowToIntake(Paths.getInstance().SOURCE_IN, true),
            GoToShoot(robotContainer, Paths.getInstance().IN_SM)
        );
    }
}
