package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicAutonGenerator;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicPathCommand;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicAutonGenerator.CompositionType;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicPathCommand.DecisionPoint;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Intake.IntakeUp;

public class DynamicPathExample extends AutonCommandBase{
    boolean skipped;
    public DynamicPathExample(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose("Test2");
        DynamicAutonGenerator auton = new DynamicAutonGenerator();
        auton.addDynamicSection(
            new DynamicPathCommand(robotContainer.getDrivetrain(), 
                getPath("Test2"), getPath("Test3"), true, 
                DecisionPoint.LIMELIGHT.setEndPoint(0.50)
            ).setSecondStop(DecisionPoint.NULL), 
            new SequentialCommandGroup(
                new IntakeIn()
            ),
            new SequentialCommandGroup(
                new IntakeUp()
            ),
            CompositionType.PARALLEL_DEADLINE
        ).addSection(
            follow("Test4"),
            new ParallelRaceGroup(
                // new ShootCommand(2000.0, 15.0),
                new WaitCommand(0.5)
            )
        );

        this.addCommands(
            auton
        );
    }
}
