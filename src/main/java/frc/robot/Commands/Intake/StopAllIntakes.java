package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Flicker;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Shooter;

public class StopAllIntakes extends Command{
    private Shooter shooter;
    private Flicker flicker;
    private Intake intake;
    private LED led;

    public StopAllIntakes(){
        this.shooter = Shooter.getInstance();
        this.flicker = Flicker.getInstance();
        this.intake = Intake.getInstance();
        this.led = LED.getInstance();

        addRequirements(shooter, flicker, intake, led);
    } 

    @Override
    public void initialize() {
        shooter.setKickerOff();
        intake.stopIntake();
        flicker.setRPM(0.0);
        led.setPrevState();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
