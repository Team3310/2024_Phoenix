package frc.robot.Auton.Middle.Center.shifted.four;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.base.CenterThreeCCN;

public class CenterFourCSSO extends AutonCommandBase{
    public CenterFourCSSO(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterThreeCCN(robotContainer),
            FollowToIntake(Paths.getInstance().CS_SIN),
            GoToShoot(robotContainer, Paths.getInstance().SIN_CS)
        );
    }
}
