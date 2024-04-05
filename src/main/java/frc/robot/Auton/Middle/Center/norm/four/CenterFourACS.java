package frc.robot.Auton.Middle.Center.norm.four;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.base.CenterThreeAIN;

public class CenterFourACS extends AutonCommandBase{
    public CenterFourACS(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterThreeAIN(robotContainer),
            FollowToIntake(Paths.getInstance().CS_CCN),
            GoToShoot(robotContainer, Paths.getInstance().CCN_CS)
        );
    }
}
