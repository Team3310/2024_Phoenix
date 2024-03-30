package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class AMThreeIN extends AutonCommandBase{
    public AMThreeIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AMTwoIN(robotContainer),
            FollowToIntake(Paths.getInstance().AS_ON),
            GoToShoot(robotContainer,Paths.getInstance().ON_AN2)
        );
    }
}
