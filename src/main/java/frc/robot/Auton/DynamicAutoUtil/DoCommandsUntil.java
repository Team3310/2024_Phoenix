package frc.robot.Auton.DynamicAutoUtil;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class DoCommandsUntil extends SequentialCommandGroup{
    public DoCommandsUntil(Command base, BooleanSupplier condition){
        this.addCommands(
            new ParallelRaceGroup(
                base,
                new WaitUntilCommand(condition)
            )
        );
    }

    public DoCommandsUntil(Command base, BooleanSupplier condition, double delay){
        this.addCommands(
            new ParallelRaceGroup(
                base,
                new WaitCommand(delay).andThen(new WaitUntilCommand(condition))
            )
        );
    }
}
