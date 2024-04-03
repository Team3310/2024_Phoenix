package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class AMFiveON extends AutonCommandBase{
    public AMFiveON(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AMFourON(robotContainer),
            FollowToIntake(Paths.getInstance().AN3_GRAB),
            AimAndShoot(robotContainer)
        );
    }
}
