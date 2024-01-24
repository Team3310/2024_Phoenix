package frc.robot.Commands.Auton;

import java.util.function.BooleanSupplier;

import frc.robot.RobotContainer;

public class TestAuton extends AutonCommandBase{
    boolean skipped;
    public TestAuton(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose("Test2");

        this.addCommands(
            new DynamicPathCommand(robotContainer.getDrivetrain(), getPath("Test2"), getPath("Test3"), true, 0.6, 0.75)
        );
    }
}
