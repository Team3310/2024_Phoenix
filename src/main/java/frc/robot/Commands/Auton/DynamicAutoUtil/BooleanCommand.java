package frc.robot.Commands.Auton.DynamicAutoUtil;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class BooleanCommand extends SequentialCommandGroup{
    public BooleanCommand(Command command, boolean end, BooleanSupplier supplier){
        if(end){
            this.addCommands(
                new ParallelRaceGroup(
                    command,
                    new WaitUntilCommand(supplier)
                )
            );
        }else{
            this.addCommands(
                new WaitUntilCommand(supplier),
                command
            );
        }
    }

    public BooleanCommand(Command command, boolean end, BooleanSupplier supplier, double waitTimeToEnd){
        if(end){
            this.addCommands(
                new ParallelRaceGroup(
                    command,
                    new SequentialCommandGroup(
                        new WaitUntilCommand(waitTimeToEnd),
                        new WaitUntilCommand(supplier)
                    )
                )
            );
        }else{
            this.addCommands(
                new WaitUntilCommand(supplier),
                command
            );
        }
    }
}
