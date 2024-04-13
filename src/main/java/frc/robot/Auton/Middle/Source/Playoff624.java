package frc.robot.Auton.Middle.Source;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Source.Base.SourceON;

public class Playoff624 extends AutonCommandBase{
    public Playoff624(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new SourceON(robotContainer),
            FollowToIntake(Paths.getInstance().SM_IN),
            GoToShoot(robotContainer, Paths.getInstance().PLAYOFFS624, false),
            FollowToIntake(Paths.getInstance().AN2_GRAB),
            AimAndShoot(robotContainer)
        );
    }
}
