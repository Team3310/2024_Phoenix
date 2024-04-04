package frc.robot.Auton.Middle.Center;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterFiveAIN extends AutonCommandBase{
    public CenterFiveAIN(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterFourAIN(robotContainer),
            FollowToIntake(Paths.getInstance().CS_SIN),
            GoToShoot(robotContainer, Paths.getInstance().SIN_CS)
        );
    }
}
