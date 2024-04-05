package frc.robot.Auton.Middle.Center.norm;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.norm.four.CenterFourSAC;

public class CenterFiveSAC extends AutonCommandBase{
    public CenterFiveSAC(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterFourSAC(robotContainer),
            FollowToIntake(Paths.getInstance().CS_CCN),
            GoToShoot(robotContainer, Paths.getInstance().CCN_CS)
        );
    }
}
