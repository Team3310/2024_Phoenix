package frc.robot.Auton.Middle.Center;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterFourSIN extends AutonCommandBase{
    public CenterFourSIN(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterThree(robotContainer),
            FollowToIntake(Paths.getInstance().CS_SIN),
            GoToShoot(robotContainer, Paths.getInstance().SIN_CS)
        );
    }
}
