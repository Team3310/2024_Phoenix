package frc.robot.Auton.Middle.Center.shifted.four;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.base.CenterThreeSIN;

public class CenterFourSCSO extends AutonCommandBase{
    public CenterFourSCSO(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterThreeSIN(robotContainer),
            FollowToIntake(Paths.getInstance().CS_CCN),
            GoToShoot(robotContainer, Paths.getInstance().CCN_CS)
        );
    }
}
