package frc.robot.Auton.Amp;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Paths;

public class TwoAmp extends AutonCommandBase{
    public TwoAmp(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new OneAuton(robotContainer),
            FollowToIntake(Paths.getInstance().TWO_AMP),
            AimAndShoot(robotContainer)
        );
    }
}
