package frc.robot.util.Choosers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.DriveTest;
import frc.robot.Commands.Auton.Amp.FourAmp;
import frc.robot.Commands.Auton.Amp.ThreeAmp;
import frc.robot.Commands.Auton.Amp.TwoAmp;
import frc.robot.Commands.Auton.Center.FourStageLeft;
import frc.robot.Commands.Auton.Center.FourStageLeftCounter;
import frc.robot.Commands.Auton.Center.ThreeStageLeft;
import frc.robot.Commands.Auton.Center.ThreeStageLeftCounter;
import frc.robot.Commands.Auton.Center.TwoStageLeft;
import frc.robot.Commands.Auton.Stage.FastStageFour;
import frc.robot.Commands.Auton.Stage.FourStage;
import frc.robot.Commands.Auton.Stage.FourStageMiddle;
import frc.robot.Commands.Auton.Stage.ThreeStage;
import frc.robot.Commands.Auton.Stage.ThreeStageMiddle;
import frc.robot.Commands.Auton.Stage.TwoStage;

public class AutonomousChooser extends ChooserBase<AutonomousChooser.AutonomousMode>{
    public AutonomousChooser() {
        super("Autonomous Mode");
        setDefaultOption(AutonomousMode.TEST);
        addOption(AutonomousMode.TWO_STAGE);
        addOption(AutonomousMode.THREE_STAGE);
        addOption(AutonomousMode.FOUR_STAGE);
        addOption(AutonomousMode.THREE_STAGE_MIDDLE);
        addOption(AutonomousMode.FOUR_STAGE_MIDDLE);
        addOption(AutonomousMode.TWO_STAGE_LEFT);
        addOption(AutonomousMode.THREE_STAGE_LEFT);
        addOption(AutonomousMode.FOUR_STAGE_LEFT);
        addOption(AutonomousMode.TWO_AMP);
        addOption(AutonomousMode.THREE_AMP);
        addOption(AutonomousMode.FOUR_AMP);
        addOption(AutonomousMode.THREE_STAGE_LEFT_COUNTER);
        addOption(AutonomousMode.FOUR_STAGE_LEFT_COUNTER);
        addOption(AutonomousMode.FAST_FOUR);
    }

    public Command getCommand() {
        return getSendable().getSelected().getCommand();
    }

    public enum AutonomousMode {
        TWO_STAGE("two stage"),
        THREE_STAGE("three stage"),
        FOUR_STAGE("four stage"),
        THREE_STAGE_MIDDLE("three stage middle"),
        FOUR_STAGE_MIDDLE("four stage middle"),
        TWO_STAGE_LEFT("two stage left"),
        THREE_STAGE_LEFT("three stage left"),
        FOUR_STAGE_LEFT("four stage left"),
        TWO_AMP("two amp"),
        THREE_AMP("three amp"),
        FOUR_AMP("four amp"),
        THREE_STAGE_LEFT_COUNTER("three stage left counter"),
        FOUR_STAGE_LEFT_COUNTER("four stage left counter"),
        FAST_FOUR("fast four"),
        TEST("drive test"),
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
                case TWO_AMP:
                    return new TwoAmp(RobotContainer.getInstance());
                case THREE_AMP:
                    return new ThreeAmp(RobotContainer.getInstance());
                case FOUR_AMP:
                    return new FourAmp(RobotContainer.getInstance());     
                case THREE_STAGE_LEFT_COUNTER:
                    return new ThreeStageLeftCounter(RobotContainer.getInstance());
                case FOUR_STAGE_LEFT_COUNTER:
                    return new FourStageLeftCounter(RobotContainer.getInstance());
                case FAST_FOUR:
                    return new FastStageFour(RobotContainer.getInstance());
                default:
                    return new DriveTest(RobotContainer.getInstance());
            }
        }
    }
}
