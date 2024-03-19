package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAuton;

public class DriveTest extends AutonCommandBase{
    public DriveTest(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().DRIVE_TEST);

        this.addCommands(
            new ParallelRaceGroup(
                Follow(Paths.getInstance().DRIVE_TEST),
                new IntakeAuton(true)
            )
        );
    }
}
