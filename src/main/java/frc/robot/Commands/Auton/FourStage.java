package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;

public class FourStage extends AutonCommandBase{
    public FourStage(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeStage(robotContainer),
            new ParallelDeadlineGroup(
                follow("4Stage"),
                new IntakeAuton()
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.1), 
                new AimLiftWithOdometryAuton()
            ),
            new FeederShootCommandAuton(robotContainer.shooter)
        );
    }
}
