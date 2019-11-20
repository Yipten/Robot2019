package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.Notifier;

public class Elevator {
	public static Notifier thread;

	public static final CANSparkMax elevator;
	public static final CANEncoder elevatorEncoder;
	public static final CANPIDController elevatorPid;
	// public static final DigitalInput halSensor;

	private static double setPosition, zeroPosition;
	private static double velocity;
	private static boolean goingUp, moving;

	// public static final double gravityFF = 0.05 * 12; // .375

	private static final double UP_ACCEL = 10_000.0;
	private static final double DOWN_ACCEL = 10_000.0;
	private static final double MAX_VELOCITY = 1000.0;
	private static final double STAGE_THRESHOLD = 30.0;
	public static final double MAX_POSITION = 47.5;
	private static final double DEADBAND = 0.26;

	/**
	 * Positions of elevator.
	 */
	public enum Position {
		ELEVATOR_FLOOR(0.0), HATCH_LEVEL_ONE(0.0), HATCH_LEVEL_TWO(23.5), HATCH_LEVEL_THREE(45.3),
		CARGO_LEVEL_ONE(17.0), CARGO_LEVEL_TWO(39.0), CARGO_LEVEL_THREE(47.0), CARGO_SHIP(33.0),
		ELEVATOR_COLLECT_CARGO(7.7),
		// ELEVATOR_COLLECT_HATCH(HATCH_DROP_OFFSET + HATCH_PLACE_OFFSET);
		ELEVATOR_COLLECT_HATCH(0.5); // ???Do we need this? TODO possibly remove?

		public double value;

		Position(double position) {
			value = position;
		}
	}

	/**
	 * Initializes the elevator motor and zeros the elevator encoder.
	 */
	static {
		elevator = new CANSparkMax(5, MotorType.kBrushless);
		elevator.setInverted(false);
		elevator.setIdleMode(IdleMode.kCoast);
		// elevator.enableVoltageCompensation(13.00); //TODO Sydney made this 13 (not
		// 12) because it is slower when charged and I think this is why? - battery
		// always starts above 12v
		elevator.setSmartCurrentLimit(60);

		elevatorEncoder = new CANEncoder(elevator);
		// halSensor = new DigitalInput(3);
		elevatorPid = new CANPIDController(elevator);
		elevatorPid.setP(0.000_02);
		elevatorPid.setI(0.000_000_1);
		elevatorPid.setD(0.000_05);
		elevatorPid.setFF(0.0);
		elevatorPid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		elevatorPid.setSmartMotionMaxAccel(UP_ACCEL, 0);
		elevatorPid.setSmartMotionMaxVelocity(MAX_VELOCITY, 0);

		elevator.setEncPosition(0);
		setPosition = 0;
		zeroPosition = 0;

		velocity = 0;
		moving = false;
	}

	/**
	 * Updates speed of elevator based on the target position.
	 */
	public static void update() {
		// if (moving) {
		// // reached set position
		// if (Math.abs(setPosition - getPosition()) <= DEADBAND) {
		// velocity = 0.0;
		// moving = false;
		// }
		// // time to slow down
		// else if (Math.abs(setPosition - getPosition()) <= getStopDistace()) {
		// velocity -= (goingUp ? UP_ACCEL : DOWN_ACCEL) * 0.02;
		// if (velocity <= 0.0) {
		// velocity = 0.0;
		// moving = false;
		// }
		// // slower than max velocity
		// } else if (velocity < MAX_VELOCITY)
		// velocity += (goingUp ? UP_ACCEL : DOWN_ACCEL) * 0.02;
		// // at max velocity
		// else if (velocity > MAX_VELOCITY)
		// velocity = MAX_VELOCITY;
		// } else
		// velocity = 0.0;
		// elevatorPid.setReference(goingUp ? velocity : -velocity,
		// ControlType.kSmartVelocity); // TODO: try using Smart
		// // Velocity

		if (Math.abs(setPosition - getPosition()) <= getStopDistace() && velocity != 0.0)
			velocity = 0.0;
		elevatorPid.setReference(velocity, ControlType.kSmartVelocity);
	}

	public static double getSetVelocity() {
		return velocity;
	}

	public static boolean isMoving() { // TODO: maybe won't actually need this
		return moving;
	}

	/**
	 * Sets the elevator to the entered position.
	 *
	 * @param position Desired elevator position.
	 */
	public static void setPosition(Position position) {
		setPosition(position.value);
	}

	/**
	 * Sets the elevator to the entered value
	 *
	 * @param targetPosition: desired elevator value
	 */
	public static void setPosition(double targetPosition) {
		setPosition = targetPosition;
		goingUp = setPosition - getPosition() > 0;
		// if (Math.abs(setPosition - getPosition()) <= getStopDistace())
		velocity = goingUp ? MAX_VELOCITY : -MAX_VELOCITY;
		// else
		// velocity = 0.0;
		moving = true;
	}

	/**
	 * Moves the elevator at a speed (percent output).
	 *
	 * @param speed: Speed of the elevator (-1.0 to 1.0).
	 */
	public static void setPercentOutput(double speed) {
		elevator.set(speed);
	}

	// /**
	// * Prevents the user from going past the max/min value of the elevator.
	// */
	// private static void limitPosition() {
	// setPosition = Math.min(setPosition, MAX_POSITION + zeroPosition);
	// setPosition = Math.max(setPosition, zeroPosition);
	// if (goingDown) {
	// elevator.getPIDController().setReference(setPosition,
	// ControlType.kSmartMotion);
	// } else {
	// elevator.getPIDController().setReference(setPosition,
	// ControlType.kSmartMotion, 0, gravityFF * 2);
	// }
	// }

	// /**
	// * Prevents the user from going past the maximum value of the elevator.
	// */
	// private static void limitPositionOverride() {
	// setPosition = Math.min(setPosition, MAX_POSITION + zeroPosition);
	// elevator.getPIDController().setReference(setPosition,
	// ControlType.kSmartMotion, 0, gravityFF);
	// }

	/**
	 * @return Position of the elevator.
	 */
	public static double getPosition() {
		return elevatorEncoder.getPosition()/* - zeroPosition */; // removed to try to fix problem switch back if manual
																	// override does a bad
	}

	/**
	 * @return Distance in rotations needed to fully stop with given velocity and
	 *         acceleration.
	 */
	public static double getStopDistace() {
		return Math.pow(getVelocity(), 2) / (2 * (goingUp ? UP_ACCEL : DOWN_ACCEL));
	}

	/**
	 * @return True if the elevator is within deadband of its set value.
	 */
	public static boolean atSetPosition() {
		return Math.abs(elevatorEncoder.getPosition() - setPosition) < DEADBAND;
	}

	/**
	 * Sets the elevator set value to its current value.
	 */
	public static void stop() {
		// setPosition(getPosition()); // if this is bad change it back
		// setPosition = elevatorEncoder.getPosition();
		// setPosition(setPosition);
		elevatorPid.setReference(0, ControlType.kVelocity);
	}

	/**
	 * Sets the elevator set value to its current value.
	 */
	public static double getCurrent() {
		return elevator.getOutputCurrent();
	}

	/**
	 * @return Set point of the elevator.
	 */
	public static double getSetPosition() {
		return setPosition;// - zeroPosition;
	}

	/**
	 * @return True if the elevator is above the stationary stage.
	 */
	public static boolean aboveStageThreshold() {
		return elevatorEncoder.getPosition() > STAGE_THRESHOLD + zeroPosition;
	}

	/**
	 * Sets the current elevator position to the new zero.
	 */
	public static void resetEncoder() {
		// elevator.setEncPosition(0);
		zeroPosition = elevatorEncoder.getPosition();
		setPosition = Position.ELEVATOR_FLOOR.value;
		setPosition(setPosition);
	}

	/**
	 * @return Belt speed in inches per second.
	 */
	public static double getBeltVelocity() {
		final double kNeoToBelt = 16.213 / 60 /* min to sec */ / 25.4 /* mm to inch */;
		return kNeoToBelt * getVelocity();
	}

	// /**
	// * Zeros the elevator if Hal Sensor is triggered, must be ran in
	// robotPeriodic.
	// */
	// public static void checkHalSensor() {
	// if (!halSensor.get()) {
	// zeroPosition = elevatorEncoder.getPosition() - 8.6; //9.6
	// setPosition = setPosition + zeroPosition;
	// limitPosition();
	// }
	// }

	public static double getVelocity() {
		return elevatorEncoder.getVelocity();
	}

	public static double getTemperature() {
		return elevator.getMotorTemperature();
	}

	public static double getPercentOutput() {
		return elevator.getAppliedOutput();
	}
}