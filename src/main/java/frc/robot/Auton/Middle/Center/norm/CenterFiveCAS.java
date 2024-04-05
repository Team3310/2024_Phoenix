package frc.robot.Auton.Middle.Center.norm;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.norm.four.CenterFourCAS;

public class CenterFiveCAS extends AutonCommandBase{
    public CenterFiveCAS(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterFourCAS(robotContainer),
            FollowToIntake(Paths.getInstance().CS_SIN),
            GoToShoot(robotContainer, Paths.getInstance().SIN_CS)
        );
    }
}
