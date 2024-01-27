package frc.robot.Commands.Auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.IntakeIn;
import frc.robot.Commands.IntakeUp;
import frc.robot.Commands.StopIntake;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicAutonGenerator;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicPathCommand;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicAutonGenerator.CompositionType;
import frc.robot.Commands.Auton.DynamicAutoUtil.DynamicPathCommand.DecisionPoint;

public class TestAuton extends AutonCommandBase{
    boolean skipped;
    public TestAuton(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose("Test2");
        DynamicAutonGenerator auton = new DynamicAutonGenerator();
        auton.addSection(
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
        );

        this.addCommands(
            auton
        );
    }
}
