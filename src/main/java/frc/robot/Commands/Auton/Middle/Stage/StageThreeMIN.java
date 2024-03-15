package frc.robot.Commands.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;

public class StageThreeMIN extends AutonCommandBase{
    public StageThreeMIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageTwoIN(robotContainer),
            FollowToIntake(Paths.getInstance().SM_CO),
            Follow(Paths.getInstance().CO_SM),
            AimAndShoot(robotContainer)
        );
    }
}
