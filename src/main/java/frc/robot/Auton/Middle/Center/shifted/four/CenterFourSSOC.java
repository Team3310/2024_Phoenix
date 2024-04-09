package frc.robot.Auton.Middle.Center.shifted.four;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.base.CenterThreeSIN;

public class CenterFourSSOC extends AutonCommandBase{
    public CenterFourSSOC(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterThreeSIN(robotContainer),
            FollowToIntake(Paths.getInstance().CS_SON),
            GoToShoot(robotContainer, Paths.getInstance().SON_CS)
        );
    }
}