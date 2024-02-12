package frc.robot.util;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.FourNote;
import frc.robot.Commands.Auton.Test;
import frc.robot.Commands.Auton.TestAuton;
import frc.robot.Commands.Auton.TestOneNote;
import frc.robot.Commands.Auton.ThreeNote;
import frc.robot.Commands.Auton.TwoNote;
import frc.robot.Commands.Auton.TwoNoteAnywhere;

public class AutonomousChooser {

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser() {
        autonomousModeChooser.setDefaultOption("Test", AutonomousMode.TEST);
        autonomousModeChooser.addOption("3 Note", AutonomousMode.THREE_NOTE);
        autonomousModeChooser.addOption("Limelight test auto", AutonomousMode.TEST_LIMELIGHT);
        autonomousModeChooser.addOption("4 Note", AutonomousMode.FOUR_NOTE);
        autonomousModeChooser.addOption("2 Note", AutonomousMode.TWO_NOTE);
        autonomousModeChooser.addOption("2 Note Anywhere", AutonomousMode.TWO_NOTE_ANYWHERE);
        autonomousModeChooser.addOption("1 Note Test", AutonomousMode.ONE_NOTE_TEST);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }


    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case FOUR_NOTE:
                return new FourNote(container);
            case THREE_NOTE:
                return new ThreeNote(container);
            case TWO_NOTE:
                return new TwoNote(container);
            case TWO_NOTE_ANYWHERE:
                return new TwoNoteAnywhere(container);
            case TEST_LIMELIGHT:
                return new TestAuton(container);  
            case ONE_NOTE_TEST:
                return new TestOneNote(container);         
            case TEST:
                //fall through
            default:
                return new Test(container);
        }
    }

    public enum AutonomousMode {
        TEST,
        TWO_NOTE,
        THREE_NOTE,
        FOUR_NOTE,
        TWO_NOTE_ANYWHERE,
        TEST_LIMELIGHT,
        ONE_NOTE_TEST,
        ;
    }
}
