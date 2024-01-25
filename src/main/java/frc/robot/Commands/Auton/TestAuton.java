package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.IntakeIn;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicAutonGenerator;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicPathCommand;

public class TestAuton extends AutonCommandBase{
    boolean skipped;
    public TestAuton(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose("Test2");
        DynamicAutonGenerator auton = new DynamicAutonGenerator();
        auton.addSection(
            new DynamicPathCommand(robotContainer.getDrivetrain(), getPath("Test2"), getPath("Test3"), true, 0.6, 0.75), 
            new InstantCommand(()->SmartDashboard.putString("ran command", "one")),
            new IntakeIn()
        );

        this.addCommands(
            auton
        );
    }
}
