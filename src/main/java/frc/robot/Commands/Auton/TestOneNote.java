package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Hood.SetHoodAngle;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Commands.Shooter.SetShooterKickerRPM;
import frc.robot.Subsystems.Hood;
import frc.robot.Subsystems.Shooter;

public class TestOneNote extends AutonCommandBase{
    public TestOneNote(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(getPath("One"));

        this.addCommands(
            new ParallelCommandGroup(
                follow("One"), 
                new IntakeAuton()
            ),
            new ParallelCommandGroup(
                new SetHoodAngle(Hood.getInstance(), 30.0),
                new SetLeftShooterRPM(Shooter.getInstance(), 6000),
                new SetRightShooterRPM(Shooter.getInstance(), 3000)
            ),
            new WaitCommand(1.0),
            new SetShooterKickerRPM(Shooter.getInstance(), 1500)
        );
    }
}
