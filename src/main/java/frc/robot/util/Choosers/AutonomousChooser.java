package frc.robot.util.Choosers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.FourStage;
import frc.robot.Commands.Auton.FourStageLeft;
import frc.robot.Commands.Auton.FourStageMiddle;
import frc.robot.Commands.Auton.Test;
import frc.robot.Commands.Auton.TestOneNote;
import frc.robot.Commands.Auton.ThreeStage;
import frc.robot.Commands.Auton.ThreeStageLeft;
import frc.robot.Commands.Auton.ThreeStageMiddle;
import frc.robot.Commands.Auton.TwoStage;
import frc.robot.Commands.Auton.TwoStageLeft;

public class AutonomousChooser extends ChooserBase<AutonomousChooser.AutonomousMode>{
    public AutonomousChooser() {
        super("Autonomous Mode");
        setDefaultOption(AutonomousMode.TEST);
        addOption(AutonomousMode.TEST_ONE);
        addOption(AutonomousMode.TWO_STAGE);
        addOption(AutonomousMode.THREE_STAGE);
        addOption(AutonomousMode.FOUR_STAGE);
        addOption(AutonomousMode.THREE_STAGE_MIDDLE);
        addOption(AutonomousMode.FOUR_STAGE_MIDDLE);
        addOption(AutonomousMode.TWO_STAGE_LEFT);
        addOption(AutonomousMode.THREE_STAGE_LEFT);
        addOption(AutonomousMode.FOUR_STAGE_LEFT);
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
        THREE_STAGE_MIDDLE("three stage middle"),
        FOUR_STAGE_MIDDLE("four stage middle"),
        TWO_STAGE_LEFT("two stage left"),
        THREE_STAGE_LEFT("three stage left"),
        FOUR_STAGE_LEFT("four stage left")
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
                case THREE_STAGE_MIDDLE:
                    return new ThreeStageMiddle(RobotContainer.getInstance());
                case FOUR_STAGE_MIDDLE:
                    return new FourStageMiddle(RobotContainer.getInstance());   
                case TWO_STAGE_LEFT:
                    return new TwoStageLeft(RobotContainer.getInstance());
                case THREE_STAGE_LEFT:
                    return new ThreeStageLeft(RobotContainer.getInstance());
                case FOUR_STAGE_LEFT:
                    return new FourStageLeft(RobotContainer.getInstance());                 
                default:
                    return new WaitCommand(0.0);
            }
        }
    }
}
