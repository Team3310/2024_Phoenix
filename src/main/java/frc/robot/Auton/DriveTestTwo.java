package frc.robot.Auton;

import frc.robot.RobotContainer;

public class DriveTestTwo extends AutonCommandBase{
    public DriveTestTwo(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().DRIVE_TEST_TWO);

        this.addCommands(
            Follow(Paths.getInstance().DRIVE_TEST_TWO)
        );
    }
}
