package frc.robot.Commands.Auton.Middle.Stage.Base;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.OneAuton;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Subsystems.Lift;

public class StageIN extends AutonCommandBase{
    public StageIN(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().SOURCE_IN);

        this.addCommands(
            new OneAuton(robotContainer),
            FollowToIntake(Paths.getInstance().SOURCE_IN),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().IN_SM),
                new SetLiftAngle(Lift.getInstance(), Constants.LIFT_MIN_DEGREES)
            ),
            AimAndShoot(robotContainer)
        );
    }
}
