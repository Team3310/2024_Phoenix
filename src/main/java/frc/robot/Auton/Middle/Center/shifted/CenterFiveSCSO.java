package frc.robot.Auton.Middle.Center.shifted;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.shifted.four.CenterFourSCSO;

public class CenterFiveSCSO extends AutonCommandBase{
    public CenterFiveSCSO(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterFourSCSO(robotContainer),
            FollowToIntake(Paths.getInstance().CS_SON),
            GoToShoot(robotContainer, Paths.getInstance().SON_CS)
        );
    }
}
