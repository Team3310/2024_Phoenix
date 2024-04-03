package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class AMFourMON extends AutonCommandBase{
    public AMFourMON(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AMTwoON(robotContainer),
            FollowToIntake(Paths.getInstance().AS_IN),
            GoToShoot(robotContainer,Paths.getInstance().IN_AS),
            FollowToIntake(Paths.getInstance().AS_CN),
            GoToShoot(robotContainer, Paths.getInstance().CN_AS, false)
        );
    }
}
