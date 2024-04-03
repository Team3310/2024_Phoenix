package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class AMFourON extends AutonCommandBase{
    public AMFourON(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AMThreeON(robotContainer),
            FollowToAmpIntake(Paths.getInstance().AN2_GRAB),
            AimAndShoot(robotContainer)
        );
    }
}
