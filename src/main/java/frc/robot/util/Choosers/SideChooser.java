package frc.robot.util.Choosers;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SideChooser extends ChooserBase<SideChooser.SideMode>{

    public SideChooser() {
        super("Side");
        setDefaultOption(SideMode.RED)
        .addOption(SideMode.BLUE);
    }

    public SideMode getSide() {
        return getSendable().getSelected();
    }

    public enum SideMode { 
        RED("Red"),
        BLUE("Blue");

        SideMode(String s){
            side=s;
        }

        String side;

        @Override
        public String toString(){
            return side;
        }

        public Alliance getAlliance() {
            switch (this) {
                case RED:
                    return Alliance.Red;
                case BLUE:
                    return Alliance.Blue;
                default:
                    return Alliance.Red;
            }
        }
    }
}