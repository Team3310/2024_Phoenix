package frc.robot.Auton.Middle.Center.norm;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.norm.four.CenterFourCSA;

public class CenterFiveCSA extends AutonCommandBase{
    public CenterFiveCSA(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterFourCSA(robotContainer),
            FollowToIntake(Paths.getInstance().CS_AIN),
            GoToShoot(robotContainer, Paths.getInstance().AIN_CS)
        );
    }
}
