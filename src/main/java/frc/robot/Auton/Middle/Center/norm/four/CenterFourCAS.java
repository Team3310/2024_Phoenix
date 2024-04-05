package frc.robot.Auton.Middle.Center.norm.four;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.base.CenterThreeCCN;

public class CenterFourCAS extends AutonCommandBase{
    public CenterFourCAS(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new CenterThreeCCN(robotContainer),
            FollowToIntake(Paths.getInstance().CS_AIN),
            GoToShoot(robotContainer, Paths.getInstance().AIN_CS)
        );
    }
}
