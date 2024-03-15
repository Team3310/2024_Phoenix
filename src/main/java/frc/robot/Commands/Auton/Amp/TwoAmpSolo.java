package frc.robot.Commands.Auton.Amp;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;

public class TwoAmpSolo extends AutonCommandBase{
    public TwoAmpSolo(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new TwoAmp(robotContainer),
            FollowToIntake(Paths.getInstance().TWO_AMP_SOLO),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().TWO_AMP_SOLO2),
            AimAndShoot(robotContainer)
        );
    }
}
