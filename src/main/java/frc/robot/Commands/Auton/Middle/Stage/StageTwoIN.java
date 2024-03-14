package frc.robot.Commands.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;

public class StageTwoIN extends AutonCommandBase{
    public StageTwoIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageIN(robotContainer),
            FollowToIntake(Paths.getInstance().SM_ON),
            Follow(Paths.getInstance().ON_SM),
            AimAndShoot(robotContainer)
        );
    }
}
