package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;

public class ElevatorAutoZero extends Command {
	private Elevator elevator;
	private double MIN_ELEVATOR_POSITION_CHANGE = 0.4;
	private double lastElevatorPosition;
	private int encoderCount;
	private double speed;

	public ElevatorAutoZero(Elevator elevator) {
		this.elevator = elevator;

		addRequirements(elevator);
	}

	@Override
	public void initialize() {
		lastElevatorPosition = Constants.ELEVATOR_MAX_INCHES;
		speed = Constants.ELEVATOR_AUTO_ZERO_SPEED;
		encoderCount = 0;
		elevator.setSpeed(speed);
		System.out.println("elevator speed = " + speed);
	}

	@Override
	public void execute() {
	}

	@Override
	public boolean isFinished() {
		double currentElevatorPosition = elevator.getPositionInches();
		double positionChange = lastElevatorPosition - currentElevatorPosition;
		lastElevatorPosition = currentElevatorPosition;
		boolean test = encoderCount > 2 && Math.abs(positionChange) < MIN_ELEVATOR_POSITION_CHANGE
				&& elevator.getCurrent() > Constants.ELEVATOR_AUTO_ZERO_MOTOR_CURRENT;
		System.out.println("encoderCount = " + encoderCount + ", test = " + test + ", elevator change = "
				+ positionChange + ", current = " + elevator.getCurrent());

		if (Math.abs(positionChange) < MIN_ELEVATOR_POSITION_CHANGE) {
			encoderCount++;
		} else {
			encoderCount = 0;
		}

		return test;
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("Elevator zero set");
		elevator.setSpeed(0);
		elevator.setElevatorZero(Constants.ELEVATOR_MIN_INCHES);
		elevator.setPosition(Constants.ELEVATOR_MIN_INCHES);
	}
}
