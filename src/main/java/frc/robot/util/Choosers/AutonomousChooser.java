package frc.robot.util.Choosers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Auton.DriveTest;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.TestGyroInit;
import frc.robot.Auton.Amp.FourAmp;
import frc.robot.Auton.Amp.ThreeAmpSolo;
import frc.robot.Auton.Amp.TwoAmpSolo;
import frc.robot.Auton.Center.FourStageLeft;
import frc.robot.Auton.Center.FourStageLeftCounter;
import frc.robot.Auton.Center.ThreeStageLeft;
import frc.robot.Auton.Center.ThreeStageLeftCounter;
import frc.robot.Auton.Center.TwoStageLeft;
import frc.robot.Auton.Middle.Amp.AMFiveIN;
import frc.robot.Auton.Middle.Amp.AMFiveON;
import frc.robot.Auton.Middle.Amp.AMFourIN;
import frc.robot.Auton.Middle.Amp.AMFourMIN;
import frc.robot.Auton.Middle.Amp.AMFourMON;
import frc.robot.Auton.Middle.Amp.AMFourON;
import frc.robot.Auton.Middle.Center.CenterFiveAIN;
import frc.robot.Auton.Middle.Center.CenterFiveSIN;
import frc.robot.Auton.Middle.Counter.Anti3005Inside;
import frc.robot.Auton.Middle.Counter.Anti3005Outside;
import frc.robot.Auton.Middle.Stage.StageThreeCNMiddle;
import frc.robot.Auton.Middle.Stage.StageThreeIN;
import frc.robot.Auton.Middle.Stage.StageThreeINMiddle;
import frc.robot.Auton.Middle.Stage.StageThreeMIN;
import frc.robot.Auton.Middle.Stage.StageThreeMON;
import frc.robot.Auton.Middle.Stage.StageThreeON;
import frc.robot.Auton.Middle.Stage.StageTwoIN;
import frc.robot.Auton.Middle.Stage.StageTwoON;
import frc.robot.Auton.Middle.Stage.Base.StageIN;
import frc.robot.Auton.Middle.Stage.Base.StageON;
import frc.robot.Auton.Stage.FastStageFour;
import frc.robot.Auton.Stage.FourStage;
import frc.robot.Auton.Stage.FourStageMiddle;
import frc.robot.Auton.Stage.ThreeStage;
import frc.robot.Auton.Stage.ThreeStageMiddle;
import frc.robot.Auton.Stage.TwoStage;

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
        .addOption(AutonomousMode.ANTI_3005O)
        .addOption(AutonomousMode.ANTI_3005I)
        .addOption(AutonomousMode.DRIVE_TEST)
        .addOption(AutonomousMode.GYRO_TEST)
        .addOption(AutonomousMode.STAGE_THREE_MMCN)
        .addOption(AutonomousMode.STAGE_THREE_MMIN)
        .addOption(AutonomousMode.AMP_FIVE_IN)
        .addOption(AutonomousMode.AMP_FIVE_ON)
        .addOption(AutonomousMode.AMP_FIVE_MIN)
        .addOption(AutonomousMode.AMP_FIVE_MON)
        .addOption(AutonomousMode.AMP_SIX_IN)
        .addOption(AutonomousMode.AMP_SIX_ON)
        .addOption(AutonomousMode.CENTER_FIVE_SIN)
        .addOption(AutonomousMode.CENTER_FIVE_AIN)
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
        STAGE_THREE_ON("stage three 1.ON->2.IN->3.PN"),
        STAGE_THREE_IN("stage three 1.IN->2.ON->3.PN"),
        STAGE_THREE_MON("stage three 1.ON->2.IN->3.CN"),
        STAGE_THREE_MIN("stage three 1.IN->2.ON->3.CN"),
        DRIVE_TEST("drive test"),
        GYRO_TEST("gyro test"),
        ANTI_3005O("anti 3005 1.outside->2.CN"),
        ANTI_3005I("anti 3005 1.inside->2.CN"),
        STAGE_THREE_MMCN("stage three 1.ON->2.CN_MM->3.IN_MM"),
        STAGE_THREE_MMIN("stage three 1.ON->2.IN_MM->3.CN_MM"),
        AMP_FIVE_ON("Amp 5 1.ON->IN"),
        AMP_FIVE_IN("Amp 5 1.IN->ON"),
        AMP_FIVE_MON("Amp 5 1.ON->IN->CN"),
        AMP_FIVE_MIN("Amp 5 1.IN->ON->CN"),
        AMP_SIX_ON("Amp 6 1.ON->IN"),
        AMP_SIX_IN("Amp 6 1.IN->ON"),
        CENTER_FIVE_SIN("Center 5 SIN"),
        CENTER_FIVE_AIN("Center 5 AIN"),
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
                case STAGE_THREE_MMCN:
                    return new StageThreeCNMiddle(RobotContainer.getInstance());
                case STAGE_THREE_MMIN:
                    return new StageThreeINMiddle(RobotContainer.getInstance());
                case ANTI_3005O:
                    return new Anti3005Outside(RobotContainer.getInstance());
                case ANTI_3005I:
                    return new Anti3005Inside(RobotContainer.getInstance());
                case AMP_FIVE_IN:
                    return new AMFourIN(RobotContainer.getInstance());
                case AMP_FIVE_ON:
                    return new AMFourON(RobotContainer.getInstance());
                case AMP_FIVE_MIN:
                    return new AMFourMIN(RobotContainer.getInstance());
                case AMP_FIVE_MON:
                    return new AMFourMON(RobotContainer.getInstance());
                case AMP_SIX_IN:
                    return new AMFiveIN(RobotContainer.getInstance());
                case AMP_SIX_ON:
                    return new AMFiveON(RobotContainer.getInstance());
                case CENTER_FIVE_AIN:
                    return new CenterFiveAIN(RobotContainer.getInstance());
                case CENTER_FIVE_SIN:
                    return new CenterFiveSIN(RobotContainer.getInstance());
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
