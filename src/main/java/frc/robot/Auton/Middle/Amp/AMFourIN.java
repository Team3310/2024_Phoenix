package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class AMFourIN extends AutonCommandBase{
    public AMFourIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AMThreeIN(robotContainer),
            FollowToIntake(Paths.getInstance().AN2_GRAB),
            AimAndShoot(robotContainer)
        );
    }
}
