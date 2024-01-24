package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SpotChooser {
    private SendableChooser<SpotMode> sideChooser = new SendableChooser<>();

    public SpotChooser() {
        sideChooser.setDefaultOption("Amp", SpotMode.AMP);
        sideChooser.addOption("Center", SpotMode.CENTER);
        sideChooser.addOption("Outside", SpotMode.OUTSIDE);
        sideChooser.addOption("Test", SpotMode.TEST);
    }

    public SendableChooser<SpotMode> getSendableChooser() {
        return sideChooser;
    }

    public SpotMode getSpot() {
        return sideChooser.getSelected();
    }

    public enum SpotMode { 
        AMP("Amp"),
        CENTER("Center"),
        OUTSIDE("Outside"),
        TEST("");

        String name;

        SpotMode(String name){
            this.name = name;
        }

        public String getName(){
            return name;
        }
    }
}
