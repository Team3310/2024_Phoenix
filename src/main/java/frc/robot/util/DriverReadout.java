package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotContainer;

public class DriverReadout {

    public DriverReadout(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");

        tab.add("Autonomous Mode", container.getAutonomousChooser().getAutonomousModeChooser())
                .withSize(2, 1)
                .withPosition(2, 0);
        tab.add("Side", container.getSideChooser())
                .withSize(2, 1)
                .withPosition(2, 1);
        tab.add("Spot", container.getSpotChooser())
                .withSize(2, 1)
                .withPosition(2, 2);        
    }
}
