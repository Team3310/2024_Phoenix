package frc.robot.util.Choosers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.FourStage;
import frc.robot.Commands.Auton.Test;
import frc.robot.Commands.Auton.TestOneNote;
import frc.robot.Commands.Auton.ThreeStage;
import frc.robot.Commands.Auton.TwoStage;

public class AutonomousChooser extends ChooserBase<AutonomousChooser.AutonomousMode>{
    public AutonomousChooser() {
        super("Autonomous Mode");
        setDefaultOption(AutonomousMode.TEST);
        addOption(AutonomousMode.TEST_ONE);
        addOption(AutonomousMode.TWO_STAGE);
        addOption(AutonomousMode.THREE_STAGE);
        addOption(AutonomousMode.FOUR_STAGE);
    }

    public Command getCommand() {
        return getSendable().getSelected().getCommand();
    }

    public enum AutonomousMode {
        TEST("test"),
        TEST_ONE("one note test"),
        TWO_STAGE("two stage"),
        THREE_STAGE("three stage"),
        FOUR_STAGE("four stage"),
        ;

        private String name = "";

        private AutonomousMode(String name){
            this.name = name;
        }

        @Override 
        public String toString(){
            return name;
        }

        public Command getCommand(){
            switch (this) {
                case FOUR_STAGE:
                    return new FourStage(RobotContainer.getInstance());
                case THREE_STAGE:
                    return new ThreeStage(RobotContainer.getInstance());
                case TWO_STAGE:
                    return new TwoStage(RobotContainer.getInstance());
                default:
                    return new WaitCommand(0.0);
            }
        }
    }
}
