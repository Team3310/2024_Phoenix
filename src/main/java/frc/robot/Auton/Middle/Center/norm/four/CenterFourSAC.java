package frc.robot.Auton.Middle.Center.norm.four;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.base.CenterThreeSIN;

public class CenterFourSAC extends AutonCommandBase{
    public CenterFourSAC(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterThreeSIN(robotContainer),
            FollowToIntake(Paths.getInstance().CS_AIN),
            GoToShoot(robotContainer, Paths.getInstance().AIN_CS)
        );
    }
}
