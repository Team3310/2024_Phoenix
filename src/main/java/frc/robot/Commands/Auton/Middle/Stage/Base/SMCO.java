package frc.robot.Commands.Auton.Middle.Stage.Base;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Subsystems.Lift;

public class SMCO extends AutonCommandBase{
    public SMCO(RobotContainer container){
        super(container);

        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_CO),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().CO_SM),
                new SetLiftAngle(Lift.getInstance(), Constants.LIFT_MIN_DEGREES)
            ),
            AimAndShoot(robotContainer)
        );
    }
}
