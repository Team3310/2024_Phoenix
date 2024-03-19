package frc.robot.Commands.Auton.Middle.Stage.Base;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.OneAuton;
import frc.robot.Commands.Auton.Paths;

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
