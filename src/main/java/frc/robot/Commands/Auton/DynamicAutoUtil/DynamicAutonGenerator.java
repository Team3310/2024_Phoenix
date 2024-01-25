package frc.robot.Commands.Auton.DynamicAutoUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicAutonGenerator extends SequentialCommandGroup{

    public DynamicAutonGenerator() {
        
    }

    public DynamicAutonGenerator addSection(DynamicPathCommand pathCommand, Command firstCommand, Command secondCommand){
        this.addCommands(
            new ParallelCommandGroup(
                pathCommand,
                new SequentialCommandGroup(
                    new BooleanCommand(firstCommand, true, ()->pathCommand.getChanged()),
                    new BooleanCommand(secondCommand, false, ()->pathCommand.getChanged())
                )
            )
        );
        return this;
    }
    
}
