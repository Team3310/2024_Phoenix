package frc.robot.Commands.Auton;

import frc.robot.RobotContainer;

public class DriveTest extends AutonCommandBase{
    public DriveTest(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().DRIVE_TEST);

        this.addCommands(
            Follow(Paths.getInstance().DRIVE_TEST),
            Follow(Paths.getInstance().DRIVE_TEST_TWO),
            Follow(Paths.getInstance().DRIVE_TEST),
            Follow(Paths.getInstance().DRIVE_TEST_TWO),
            Follow(Paths.getInstance().DRIVE_TEST),
            Follow(Paths.getInstance().DRIVE_TEST_TWO),
            Follow(Paths.getInstance().DRIVE_TEST),
            Follow(Paths.getInstance().DRIVE_TEST_TWO),
            Follow(Paths.getInstance().DRIVE_TEST),
            Follow(Paths.getInstance().DRIVE_TEST_TWO),
            Follow(Paths.getInstance().DRIVE_TEST),
            Follow(Paths.getInstance().DRIVE_TEST_TWO)
        );
    }
}
