package frc.robot.util.Choosers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Auton.DriveTest;
import frc.robot.Auton.DriveTestTwo;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.TestGyroInit;
import frc.robot.Auton.Amp.FourAmp;
import frc.robot.Auton.Amp.ThreeAmpSolo;
import frc.robot.Auton.Amp.TwoAmpSolo;
import frc.robot.Auton.Middle.Amp.AmpIOAN2;
import frc.robot.Auton.Middle.Amp.AmpIOAN2AN3;
import frc.robot.Auton.Middle.Amp.AmpIOC;
import frc.robot.Auton.Middle.Amp.AmpOCI;
import frc.robot.Auton.Middle.Amp.AmpOIAN2;
import frc.robot.Auton.Middle.Amp.AmpOIAN2AN3;
import frc.robot.Auton.Middle.Amp.AmpOIC;
import frc.robot.Auton.Middle.Center.CenterFive;
import frc.robot.Auton.Middle.Center.CenterFiveAmp;
import frc.robot.Auton.Middle.Center.CenterFiveSource;
import frc.robot.Auton.Middle.Center.norm.CenterFiveACS;
import frc.robot.Auton.Middle.Center.norm.CenterFiveASC;
import frc.robot.Auton.Middle.Center.norm.CenterFiveCAS;
import frc.robot.Auton.Middle.Center.norm.CenterFiveCSA;
import frc.robot.Auton.Middle.Center.norm.CenterFiveSAC;
import frc.robot.Auton.Middle.Center.norm.CenterFiveSCA;
import frc.robot.Auton.Middle.Center.over.CenterFiveOver;
import frc.robot.Auton.Middle.Center.over.CenterOverCAS;
import frc.robot.Auton.Middle.Center.over.CenterOverCSA;
import frc.robot.Auton.Middle.Center.shifted.CenterFiveCSOS;
import frc.robot.Auton.Middle.Center.shifted.CenterFiveCSSO;
import frc.robot.Auton.Middle.Center.shifted.CenterFiveSCSO;
import frc.robot.Auton.Middle.Center.shifted.CenterFiveSSOC;
import frc.robot.Auton.Middle.Source.Playoff624;
import frc.robot.Auton.Middle.Source.SourceICO;
import frc.robot.Auton.Middle.Source.SourceICP;
import frc.robot.Auton.Middle.Source.SourceIOC;
import frc.robot.Auton.Middle.Source.SourceIOP;
import frc.robot.Auton.Middle.Source.SourceOCI;
import frc.robot.Auton.Middle.Source.SourceOCP;
import frc.robot.Auton.Middle.Source.SourceOIC;
import frc.robot.Auton.Middle.Source.SourceOIP;
import frc.robot.Auton.Middle.Source.Base.SMCNPN.CNPNPath;
import frc.robot.Auton.Source.FourSource;
import frc.robot.Auton.Source.ThreeSource;
import frc.robot.Auton.Source.TwoSource;

public class AutonomousChooser extends ChooserBase<AutonomousChooser.AutonomousMode>{
    public AutonomousChooser() {
        super("Autonomous Mode");
        setDefaultOption(AutonomousMode.ONE_AUTON)
        //#region source
        .addOption(AutonomousMode.TWO_SOURCE)
        .addOption(AutonomousMode.THREE_SOURCE)
        .addOption(AutonomousMode.FOUR_SOURCE)
        .addOption(AutonomousMode.SOURCE_OIC)
        .addOption(AutonomousMode.SOURCE_OIP)
        .addOption(AutonomousMode.SOURCE_OCP)
        .addOption(AutonomousMode.SOURCE_OCPS)
        .addOption(AutonomousMode.SOURCE_OCI)
        .addOption(AutonomousMode.SOURCE_IOC)
        .addOption(AutonomousMode.SOURCE_ICP)
        .addOption(AutonomousMode.SOURCE_ICO)
        .addOption(AutonomousMode.SOURCE_ICPS)
        .addOption(AutonomousMode.SOURCE_IOP)
        //#endregion
        //#region amp
        .addOption(AutonomousMode.TWO_AMP)
        .addOption(AutonomousMode.THREE_AMP)
        .addOption(AutonomousMode.FOUR_AMP)
        .addOption(AutonomousMode.AMP_OIC)
        .addOption(AutonomousMode.AMP_OCI)
        .addOption(AutonomousMode.AMP_OIAN2)
        .addOption(AutonomousMode.AMP_OIAN2AN3)
        .addOption(AutonomousMode.AMP_IOC)
        .addOption(AutonomousMode.AMP_IOAN2)
        .addOption(AutonomousMode.AMP_IOAN2AN3)
        //#endregion
        //#region center
        .addOption(AutonomousMode.CENTER_CSA)
        .addOption(AutonomousMode.CENTER_CAS)
        .addOption(AutonomousMode.CENTER_SCA)
        .addOption(AutonomousMode.CENTER_SAC)
        .addOption(AutonomousMode.CENTER_ASC)
        .addOption(AutonomousMode.CENTER_ACS)
        .addOption(AutonomousMode.CENTER_CSSO)
        .addOption(AutonomousMode.CENTER_CSOS)
        .addOption(AutonomousMode.CENTER_SCSO)
        .addOption(AutonomousMode.CENTER_SSOC)
        .addOption(AutonomousMode.CENTER_FIVE)
        .addOption(AutonomousMode.CENTER_FIVE_OVER)
        .addOption(AutonomousMode.CENTER_OVER_CSA)
        .addOption(AutonomousMode.CENTER_OVER_CAS)
        .addOption(AutonomousMode.CENTER_FIVE_AMP)
        .addOption(AutonomousMode.CENTER_FIVE_SOURCE)
        //#endregion
        //#region counter
        .addOption(AutonomousMode.PLAYOFFS624)
        //#endregion
        //#region test
        .addOption(AutonomousMode.DRIVE_TEST)
        .addOption(AutonomousMode.DRIVE_TEST_FB)
        .addOption(AutonomousMode.GYRO_TEST)
        //#endregion
        ;
    }

    public Command getCommand() {
        return getSendable().getSelected().getCommand();
    }

    public enum AutonomousMode {
        ONE_AUTON("one note anywhere"),
        //#region source
        TWO_SOURCE("two source"),
        THREE_SOURCE("three source"),
        FOUR_SOURCE("four source"),
        SOURCE_OIC("source OIC"),
        SOURCE_OIP("source OIP"),
        SOURCE_OCI("source OCI"),
        SOURCE_OCP("source OCP"),
        SOURCE_OCPS("source OCP under stage"),
        SOURCE_IOC("source IOC"),
        SOURCE_IOP("source IOP"),
        SOURCE_ICP("source ICP"),
        SOURCE_ICO("source ICO"),
        SOURCE_ICPS("source ICP under stage"),
        //#endregion
        //#region amp
        AMP_IOC("amp IOC"),
        AMP_IOAN2("amp IOAN2"),
        AMP_IOAN2AN3("amp IOAN2AN3"),
        AMP_OIC("amp OIC"),
        AMP_OCI("amp OCI"),
        AMP_OIAN2("amp OIAN2"),
        AMP_OIAN2AN3("amp OIAN2AN3"),
        TWO_AMP("two amp"),
        THREE_AMP("three amp"),
        FOUR_AMP("four amp"),
        //#endregion
        //#region center
        CENTER_CSA("Center CSA"),
        CENTER_CAS("Center CAS"),
        CENTER_SCA("Center SCA"),
        CENTER_SAC("Center SAC"),
        CENTER_ASC("Center ASC"),
        CENTER_ACS("Center ACS"),
        CENTER_CSSO("Center CSSO"),
        CENTER_CSOS("Center CSOS"),
        CENTER_SCSO("Center SCSO"),
        CENTER_SSOC("Center SSOC"),
        CENTER_FIVE("Center Five Center"),
        CENTER_FIVE_OVER("Center Five Center Over"),
        CENTER_OVER_CSA("Center Over CSA"),
        CENTER_OVER_CAS("Center Over CAS"),
        CENTER_FIVE_AMP("Center Five Amp"),
        CENTER_FIVE_SOURCE("Center Five Source"),
        //#endregion
        //#region test
        DRIVE_TEST("drive test forward and back"),
        DRIVE_TEST_FB("drive test"),
        GYRO_TEST("gyro test"),
        //#endregion
        //#region counter  
        PLAYOFFS624("624 playoffs"),
        //#endregion
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
                //#region source
                case FOUR_SOURCE:
                    return new FourSource(RobotContainer.getInstance());
                case THREE_SOURCE:
                    return new ThreeSource(RobotContainer.getInstance());
                case TWO_SOURCE:
                    return new TwoSource(RobotContainer.getInstance()); 
                case SOURCE_OIP:
                    return new SourceOIP(RobotContainer.getInstance());
                case SOURCE_OCP:
                    return new SourceOCP(RobotContainer.getInstance(), CNPNPath.CN);
                case SOURCE_OCPS:
                    return new SourceOCP(RobotContainer.getInstance(), CNPNPath.CCNS);
                case SOURCE_OIC:
                    return new SourceOIC(RobotContainer.getInstance());
                case SOURCE_OCI:
                    return new SourceOCI(RobotContainer.getInstance());
                case SOURCE_IOP:
                    return new SourceIOP(RobotContainer.getInstance());
                case SOURCE_IOC:
                    return new SourceIOC(RobotContainer.getInstance());
                case SOURCE_ICP:
                    return new SourceICP(RobotContainer.getInstance(), CNPNPath.CN);
                case SOURCE_ICO:
                    return new SourceICO(RobotContainer.getInstance());
                case SOURCE_ICPS:
                    return new SourceICP(RobotContainer.getInstance(), CNPNPath.CCNS);
                //#endregion
                //#region amp
                case TWO_AMP:
                    return new TwoAmpSolo(RobotContainer.getInstance());
                case THREE_AMP:
                    return new ThreeAmpSolo(RobotContainer.getInstance());
                case FOUR_AMP:
                    return new FourAmp(RobotContainer.getInstance());
                case AMP_IOC:
                    return new AmpIOC(RobotContainer.getInstance());
                case AMP_IOAN2:
                    return new AmpIOAN2(RobotContainer.getInstance());
                case AMP_IOAN2AN3:
                    return new AmpIOAN2AN3(RobotContainer.getInstance());
                case AMP_OCI:
                    return new AmpOCI(RobotContainer.getInstance());
                case AMP_OIC:
                    return new AmpOIC(RobotContainer.getInstance());
                case AMP_OIAN2:
                    return new AmpOIAN2(RobotContainer.getInstance());
                case AMP_OIAN2AN3:
                    return new AmpOIAN2AN3(RobotContainer.getInstance());
                //#endregion
                //#region center
                case CENTER_CAS:
                    return new CenterFiveCAS(RobotContainer.getInstance());
                case CENTER_CSA:
                    return new CenterFiveCSA(RobotContainer.getInstance());
                case CENTER_SAC:
                    return new CenterFiveSAC(RobotContainer.getInstance());
                case CENTER_SCA:
                    return new CenterFiveSCA(RobotContainer.getInstance());
                case CENTER_ASC:
                    return new CenterFiveASC(RobotContainer.getInstance());
                case CENTER_ACS:
                    return new CenterFiveACS(RobotContainer.getInstance());
                case CENTER_CSOS:
                    return new CenterFiveCSOS(RobotContainer.getInstance());
                case CENTER_CSSO:
                    return new CenterFiveCSSO(RobotContainer.getInstance());
                case CENTER_SSOC:
                    return new CenterFiveSSOC(RobotContainer.getInstance());
                case CENTER_SCSO:
                    return new CenterFiveSCSO(RobotContainer.getInstance());
                case CENTER_FIVE:
                    return new CenterFive(RobotContainer.getInstance());
                case CENTER_FIVE_OVER:
                    return new CenterFiveOver(RobotContainer.getInstance());
                case CENTER_OVER_CAS:
                    return new CenterOverCAS(RobotContainer.getInstance());
                case CENTER_OVER_CSA:
                    return new CenterOverCSA(RobotContainer.getInstance());
                case CENTER_FIVE_AMP:
                    return new CenterFiveAmp(RobotContainer.getInstance());
                case CENTER_FIVE_SOURCE:
                    return new CenterFiveSource(RobotContainer.getInstance());
                //#endregion
                //#region counter
                case PLAYOFFS624:
                    return new Playoff624(RobotContainer.getInstance()); 
                //#endregion
                //#region test
                case GYRO_TEST:
                    return new TestGyroInit(RobotContainer.getInstance());
                case DRIVE_TEST:
                    return new DriveTest(RobotContainer.getInstance());
                case DRIVE_TEST_FB:
                    return new DriveTestTwo(RobotContainer.getInstance());
                //#endregion
                case ONE_AUTON:
                    return new OneAuton(RobotContainer.getInstance());
                default:
                    return new OneAuton(RobotContainer.getInstance());
            }
        }
    }
}
