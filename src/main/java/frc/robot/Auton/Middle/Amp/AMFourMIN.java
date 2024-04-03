package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class AMFourMIN extends AutonCommandBase{
    public AMFourMIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AMTwoIN(robotContainer),
            FollowToIntake(Paths.getInstance().AS_ON),
            GoToShoot(robotContainer,Paths.getInstance().ON_AS),
            FollowToIntake(Paths.getInstance().AS_CN),
            GoToShoot(robotContainer, Paths.getInstance().CN_AS, false)
        );
    }
}
