package frc.robot.util.Choosers;

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
    }
}