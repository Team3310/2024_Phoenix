package frc.robot.Auton.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class TwoAmpSolo extends AutonCommandBase{
    public TwoAmpSolo(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new TwoAmp(robotContainer),
            // FollowToIntake(Paths.getInstance().AN_ON),
            // GoToShoot(robotContainer, Paths.getInstance().ON_AS),
            // FollowToIntake(Paths.getInstance().AS_IN),
            // GoToShoot(robotContainer, Paths.getInstance().IN_AS, false)
            FollowToIntake(Paths.getInstance().TWO_AMP_SOLO),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().TWO_AMP_SOLO2),
            AimAndShoot(robotContainer)
        );
    }
}
