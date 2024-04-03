package frc.robot.Auton.Middle.Center;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Paths;

public class CenterTwo extends AutonCommandBase{
    public CenterTwo(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().C_N);

        this.addCommands(
            new OneAuton(robotContainer),
            FollowToIntake(Paths.getInstance().C_N),
            AimAndShoot(robotContainer)
        );
    }
}
