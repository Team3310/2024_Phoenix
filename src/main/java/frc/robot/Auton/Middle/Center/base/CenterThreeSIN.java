package frc.robot.Auton.Middle.Center.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterThreeSIN extends AutonCommandBase{
    public CenterThreeSIN(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterTwo(robotContainer),
            FollowToIntake(Paths.getInstance().CN_SIN),
            GoToShoot(robotContainer, Paths.getInstance().SIN_CS)
        );
    }
}
