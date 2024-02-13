package frc.robot.util.Choosers;

public class SpotChooser extends ChooserBase<SpotChooser.SpotMode>{

    public SpotChooser() {
        super("Spot");
        setDefaultOption(SpotMode.AMP)
        .addOption(SpotMode.CENTER)
        .addOption(SpotMode.OUTSIDE)
        .addOption(SpotMode.TEST);
    }

    public SpotMode getSpot() {
        return getSendable().getSelected();
    }

    public enum SpotMode { 
        AMP("Amp"),
        CENTER("Center"),
        OUTSIDE("Outside"),
        TEST("Test");

        String name;

        SpotMode(String name){
            this.name = name;
        }

        @Override
        public String toString(){
            return name;
        }
    }
}
