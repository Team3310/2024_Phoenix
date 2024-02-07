package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Commands.Shooter.SetShooterKickerRPM;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;

public class TestOneNote extends AutonCommandBase{
    public TestOneNote(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(getPath("One"));

        this.addCommands(
            new ParallelDeadlineGroup(
                follow("One"), 
                new IntakeAuton(),
                new SetLeftShooterRPM(Shooter.getInstance(), 6000),
                new SetRightShooterRPM(Shooter.getInstance(), 3000)
            ),
            new SetLiftAngle(Lift.getInstance(), 30.0),
            new WaitCommand(0.5),
            new SetShooterKickerRPM(Shooter.getInstance(), 1500)
        );
    }
}
