package frc.robot.Commands.Auton.DynamicAutoUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicAutonGenerator extends SequentialCommandGroup{
    public DynamicAutonGenerator addDynamicSection(DynamicPathCommand pathCommand, Command firstCommand, Command secondCommand, CompositionType type){
        switch (type) {
            case PARALLEL:
                this.addCommands(
                    new ParallelCommandGroup(
                        pathCommand,
                        new SequentialCommandGroup(
                            new BooleanCommand(firstCommand, true, ()->pathCommand.getChanged()),
                            new BooleanCommand(secondCommand, false, ()->pathCommand.getChanged())
                        )
                    )
                );
                break;
            case PARALLEL_RACE:
                this.addCommands(
                    new ParallelRaceGroup(
                        pathCommand,
                        new SequentialCommandGroup(
                            new BooleanCommand(firstCommand, true, ()->pathCommand.getChanged()),
                            new BooleanCommand(secondCommand, false, ()->pathCommand.getChanged()),
                            new InstantCommand(()->pathCommand.cancel())
                        )
                    )
                );
                break;
            case PARALLEL_DEADLINE:
                //fall through
            default:
                this.addCommands(
                    new ParallelDeadlineGroup(
                        pathCommand,
                        new SequentialCommandGroup(
                            new BooleanCommand(firstCommand, true, ()->pathCommand.getChanged()),
                            new BooleanCommand(secondCommand, false, ()->pathCommand.getChanged())
                        )
                    )
                );
                break;
        }
        return this;
    }

    public DynamicAutonGenerator addSection(Command... command){
        this.addCommands(command);
        return this;
    }

    public enum CompositionType{
        PARALLEL,
        PARALLEL_RACE,
        PARALLEL_DEADLINE,
    }
}
