package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Flicker;

public class SetElevatorInches extends Command{
    private Elevator elevator;
    private double inches;


    public SetElevatorInches(Elevator elevator, double inches){
        this.elevator = elevator;
        this.inches = inches;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        elevator.setPosition(inches);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}
