package frc.robot.Commands.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.OneAuton;
import frc.robot.Commands.Auton.Paths;

public class StageTwoON extends AutonCommandBase{
    public StageTwoON(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().STAGE_SM);

        this.addCommands(
            // new StageIN(robotContainer),
                
                    // new OneAuton(robotContainer, Paths.getInstance().STAGE_SM),
                        // resetRobotPose(Paths.getInstance().STAGE_SM); (moved outside addCommands())
                        AimAndShoot(robotContainer),
                    // end

                Follow(Paths.getInstance().STAGE_SM),
                FollowToIntake(Paths.getInstance().SOURCE_IN),
                AimAndShoot(robotContainer),
            // end

            FollowToIntake(Paths.getInstance().SOURCE_ON),
            AimAndShoot(robotContainer)
        );
    }
}
