package frc.robot.Auton.Middle.Center.shifted;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.shifted.four.CenterFourCSSO;

public class CenterFiveCSSO extends AutonCommandBase{
    public CenterFiveCSSO(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterFourCSSO(robotContainer),
            FollowToIntake(Paths.getInstance().CS_SON),
            GoToShoot(robotContainer, Paths.getInstance().SON_CS)
        );
    }
}
