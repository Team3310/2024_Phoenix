package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class AMFiveIN extends AutonCommandBase{
    public AMFiveIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AMFourIN(robotContainer),
            FollowToIntake(Paths.getInstance().AN3_GRAB),
            AimAndShoot(robotContainer)
        );
    }
}
