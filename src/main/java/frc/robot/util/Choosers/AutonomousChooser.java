package frc.robot.util.Choosers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.Test;
import frc.robot.Commands.Auton.TestOneNote;

public class AutonomousChooser extends ChooserBase<AutonomousChooser.AutonomousMode>{
    public AutonomousChooser() {
        super("Autonomous Mode");
        setDefaultOption(AutonomousMode.TEST);
        addOption(AutonomousMode.TEST_ONE);
    }

    public Command getCommand() {
        return getSendable().getSelected().getCommand();
    }

    public enum AutonomousMode {
        TEST("test", new Test(RobotContainer.getInstance())),
        TEST_ONE("one note test", new TestOneNote(RobotContainer.getInstance()))
        ;

        private Command command = new WaitCommand(0.0);
        private String name = "";

        private AutonomousMode(String name, Command command){
            this.name = name;
            this.command = command;
        }

        @Override 
        public String toString(){
            return name;
        }

        public Command getCommand(){
            return command;
        }
    }
}
