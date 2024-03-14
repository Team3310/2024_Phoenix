package frc.robot.Commands.Auton.Middle.Stage;

import edu.wpi.first.wpilibj2.command.WaitCommand;
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
                        // AimAndShoot(robotContainer),
                    // end

                Follow(Paths.getInstance().STAGE_SM), //toFarMiddleStage
                new WaitCommand(2.0),
                // FollowToIntake(Paths.getInstance().SOURCE_IN),
                Follow(Paths.getInstance().SOURCE_ON), //sourceToOn
                new WaitCommand(2.0),
                // AimAndShoot(robotContainer),
            // end

            // FollowToIntake(Paths.getInstance().SOURCE_ON)
            Follow(Paths.getInstance().SOURCE_IN) //sourcetoIn
            // AimAndShoot(robotContainer)
        );
    }
}
