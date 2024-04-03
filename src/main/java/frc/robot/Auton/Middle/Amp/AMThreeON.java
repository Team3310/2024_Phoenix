package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class AMThreeON extends AutonCommandBase{
    public AMThreeON(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AMTwoON(robotContainer),
            FollowToIntake(Paths.getInstance().AS_IN),
            GoToShoot(robotContainer,Paths.getInstance().IN_AN2, false)
        );
    }
}
