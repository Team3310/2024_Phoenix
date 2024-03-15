package frc.robot.util.Choosers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.DriveTest;
import frc.robot.Commands.Auton.OneAuton;
import frc.robot.Commands.Auton.TestGyroInit;
import frc.robot.Commands.Auton.Amp.FourAmp;
import frc.robot.Commands.Auton.Amp.ThreeAmpSolo;
import frc.robot.Commands.Auton.Amp.TwoAmpSolo;
import frc.robot.Commands.Auton.Center.FourStageLeft;
import frc.robot.Commands.Auton.Center.FourStageLeftCounter;
import frc.robot.Commands.Auton.Center.ThreeStageLeft;
import frc.robot.Commands.Auton.Center.ThreeStageLeftCounter;
import frc.robot.Commands.Auton.Center.TwoStageLeft;
import frc.robot.Commands.Auton.Middle.Stage.StageIN;
import frc.robot.Commands.Auton.Middle.Stage.StageON;
import frc.robot.Commands.Auton.Middle.Stage.StageThreeIN;
import frc.robot.Commands.Auton.Middle.Stage.StageThreeMIN;
import frc.robot.Commands.Auton.Middle.Stage.StageThreeMON;
import frc.robot.Commands.Auton.Middle.Stage.StageThreeON;
import frc.robot.Commands.Auton.Middle.Stage.StageTwoIN;
import frc.robot.Commands.Auton.Middle.Stage.StageTwoON;
import frc.robot.Commands.Auton.Stage.FastStageFour;
import frc.robot.Commands.Auton.Stage.FourStage;
import frc.robot.Commands.Auton.Stage.FourStageMiddle;
import frc.robot.Commands.Auton.Stage.ThreeStage;
import frc.robot.Commands.Auton.Stage.ThreeStageMiddle;
import frc.robot.Commands.Auton.Stage.TwoStage;

public class AutonomousChooser extends ChooserBase<AutonomousChooser.AutonomousMode>{
    public AutonomousChooser() {
        super("Autonomous Mode");
        setDefaultOption(AutonomousMode.ONE_AUTON)
        .addOption(AutonomousMode.TWO_STAGE)
        .addOption(AutonomousMode.THREE_STAGE)
        .addOption(AutonomousMode.FOUR_STAGE)
        // .addOption(AutonomousMode.THREE_STAGE_MIDDLE)
        // .addOption(AutonomousMode.FOUR_STAGE_MIDDLE)
        // .addOption(AutonomousMode.TWO_STAGE_LEFT)
        // .addOption(AutonomousMode.THREE_STAGE_LEFT)
        // .addOption(AutonomousMode.FOUR_STAGE_LEFT)
        .addOption(AutonomousMode.TWO_AMP)
        .addOption(AutonomousMode.THREE_AMP)
        .addOption(AutonomousMode.FOUR_AMP)
        // .addOption(AutonomousMode.THREE_STAGE_LEFT_COUNTER)
        // .addOption(AutonomousMode.FOUR_STAGE_LEFT_COUNTER)
        .addOption(AutonomousMode.FAST_FOUR)
        .addOption(AutonomousMode.STAGE_IN)
        .addOption(AutonomousMode.STAGE_ON)
        .addOption(AutonomousMode.STAGE_TWO_IN)
        .addOption(AutonomousMode.STAGE_TWO_ON)
        .addOption(AutonomousMode.STAGE_THREE_IN)
        .addOption(AutonomousMode.STAGE_THREE_ON)
        .addOption(AutonomousMode.STAGE_THREE_MIN)
        .addOption(AutonomousMode.STAGE_THREE_MON)
        .addOption(AutonomousMode.DRIVE_TEST)
        .addOption(AutonomousMode.GYRO_TEST)
        ;
    }

    public Command getCommand() {
        return getSendable().getSelected().getCommand();
    }

    public enum AutonomousMode {
        ONE_AUTON("one note anywhere"),
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
        STAGE_ON("stage ON"),
        STAGE_IN("stage IN"),
        STAGE_TWO_ON("stage two 1.ON->2.IN"),
        STAGE_TWO_IN("stage two 1.IN->2.ON"),
        STAGE_THREE_ON("stage three 1.ON->2.IN->"),
        STAGE_THREE_IN("stage three 1.IN->2.ON->"),
        STAGE_THREE_MON("stage three 1.ON->2.IN->3.CN"),
        STAGE_THREE_MIN("stage three 1.IN->2.ON->3.CN"),
        DRIVE_TEST("drive test"),
        GYRO_TEST("gyro test"),
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
                    return new TwoAmpSolo(RobotContainer.getInstance());
                case THREE_AMP:
                    return new ThreeAmpSolo(RobotContainer.getInstance());
                case FOUR_AMP:
                    return new FourAmp(RobotContainer.getInstance());     
                case THREE_STAGE_LEFT_COUNTER:
                    return new ThreeStageLeftCounter(RobotContainer.getInstance());
                case FOUR_STAGE_LEFT_COUNTER:
                    return new FourStageLeftCounter(RobotContainer.getInstance());
                case FAST_FOUR:
                    return new FastStageFour(RobotContainer.getInstance());
                case STAGE_IN:
                    return new StageIN(RobotContainer.getInstance());
                case STAGE_ON:
                    return new StageON(RobotContainer.getInstance());
                case STAGE_TWO_ON:
                    return new StageTwoON(RobotContainer.getInstance());
                case STAGE_TWO_IN:
                    return new StageTwoIN(RobotContainer.getInstance());
                case STAGE_THREE_IN:
                    return new StageThreeIN(RobotContainer.getInstance());
                case STAGE_THREE_ON:
                    return new StageThreeON(RobotContainer.getInstance());
                case STAGE_THREE_MON:
                    return new StageThreeMON(RobotContainer.getInstance());
                case STAGE_THREE_MIN:
                    return new StageThreeMIN(RobotContainer.getInstance());
                case GYRO_TEST:
                    return new TestGyroInit(RobotContainer.getInstance());
                case DRIVE_TEST:
                    return new DriveTest(RobotContainer.getInstance());
                case ONE_AUTON:
                    return new OneAuton(RobotContainer.getInstance());
                default:
                    return new OneAuton(RobotContainer.getInstance());
            }
        }
    }
}
