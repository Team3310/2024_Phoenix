package frc.robot.Auton.Middle.Center.shifted;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.shifted.four.CenterFourCSOS;

public class CenterFiveCSOS extends AutonCommandBase{
    public CenterFiveCSOS(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterFourCSOS(robotContainer),
            FollowToIntake(Paths.getInstance().CS_SIN),
            GoToShoot(robotContainer, Paths.getInstance().SIN_CS)
        );
    }
}
