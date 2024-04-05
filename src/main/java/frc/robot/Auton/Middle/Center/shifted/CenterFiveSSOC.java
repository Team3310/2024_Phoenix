package frc.robot.Auton.Middle.Center.shifted;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.shifted.four.CenterFourSSOC;

public class CenterFiveSSOC extends AutonCommandBase{
    public CenterFiveSSOC(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterFourSSOC(robotContainer),
            FollowToIntake(Paths.getInstance().CS_CCN),
            GoToShoot(robotContainer, Paths.getInstance().CCN_CS)
        );
    }
}
