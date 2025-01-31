package frc.robot.Auton;

import frc.robot.RobotContainer;

public class DriveTest extends AutonCommandBase{
    public DriveTest(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().DRIVE_TEST);

        this.addCommands(
            Follow(Paths.getInstance().DRIVE_TEST)
        );
    }
}
