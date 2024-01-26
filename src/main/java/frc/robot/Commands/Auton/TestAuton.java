package frc.robot.Commands.Auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
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
                DecisionPoint.TIME.setStartPoint(1.75).setEndPoint(getPathTime("Test2"))
            ).setSecondStop(DecisionPoint.TIME.setStartPoint(1.25).setEndPoint(2.5)), 
            new SequentialCommandGroup(
                new InstantCommand(()->SmartDashboard.putNumber("index", 0)),
                new WaitCommand(0.5),
                new InstantCommand(()->SmartDashboard.putNumber("index", 0.5)),
                new WaitCommand(0.5),
                new InstantCommand(()->SmartDashboard.putNumber("index", 1.0)),
                new WaitCommand(0.5),
                new InstantCommand(()->SmartDashboard.putNumber("index", 1.5)),
                new WaitCommand(0.5),
                new InstantCommand(()->SmartDashboard.putNumber("index", 2.0)),
                new WaitCommand(0.5),
                new InstantCommand(()->SmartDashboard.putNumber("index", 2.5))
            ),
            new SequentialCommandGroup(
                new InstantCommand(()->SmartDashboard.putNumber("index2", 0)),
                new WaitCommand(0.5),
                new InstantCommand(()->SmartDashboard.putNumber("index2", 0.5)),
                new WaitCommand(0.5),
                new InstantCommand(()->SmartDashboard.putNumber("index2", 1.0)),
                new InstantCommand(()->SmartDashboard.putNumber("index2", 999))
            ),
            CompositionType.PARALLEL_RACE
        );

        this.addCommands(
            auton
        );
    }
}
