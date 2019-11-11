package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;

public class Elevator {
    public static Notifier thread;

    public static final CANSparkMax elevator;
    public static final CANEncoder elevatorEncoder;
    // public static final DigitalInput halSensor;

    private static double setPosition, prevPosition, zeroPosition;
    private static double distance;
    private static boolean goingUp, atMaxSpeed, finished;
    // public static final double gravityFF = 0.05 * 12; // .375

    private static final double UP_ACCEL = 0.1; // increase in percent output per encoder tick TODO: find actual value
    private static final double DOWN_ACCEL = 0.1; // TODO: find actual value
    private static final double STAGE_THRESHOLD = 30.0;
    public static final double MAX_POSITION = 47.5;
    private static final double DEADBAND = 0.26;

    /**
     * Positions of elevator
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
     * Initializes the elevator motor and zeros the elevator encoder
     */
    static {
        elevator = new CANSparkMax(5, MotorType.kBrushless);
        elevator.setInverted(false);
        elevator.setIdleMode(IdleMode.kBrake);
        // elevator.enableVoltageCompensation(13.00); //TODO Sydney made this 13 (not
        // 12) because it is slower when charged and I think this is why? - battery
        // always starts above 12v
        elevator.setSmartCurrentLimit(60);

        elevatorEncoder = new CANEncoder(elevator);
        // halSensor = new DigitalInput(3);

        elevator.setEncPosition(0);
        setPosition = elevatorEncoder.getPosition();
        prevPosition = elevatorEncoder.getPosition();
        zeroPosition = elevatorEncoder.getPosition();

        thread = new Notifier(() -> update());
    }

    public static void startThread() {
        thread.startPeriodic(0.02);
    }

    public static void stopThread() {
        thread.stop();
    }

    /**
     * Updates speed of elevator based on setpoint
     */
    public static void update() {
        if (!finished)
            if (goingUp) {
                if (!atMaxSpeed) {
                    setPercentOutput(getPercentOutput() + UP_ACCEL * getPositionDiff());
                    if (getPercentOutput() >= 1.0) {
                        setPercentOutput(1.0);
                        atMaxSpeed = true;
                    }
                } else if (Math.abs(setPosition - getPosition()) <= 1.0 / UP_ACCEL) {
                    setPercentOutput(getPercentOutput() - UP_ACCEL * getPositionDiff());
                    if (getPercentOutput() <= 0.0) {
                        setPercentOutput(0.0);
                        finished = true;
                    }
                }
            } else {
                if (!atMaxSpeed) {
                    setPercentOutput(getPercentOutput() + DOWN_ACCEL * getPositionDiff());
                    if (getPercentOutput() <= -1.0) {
                        setPercentOutput(-1.0);
                        atMaxSpeed = true;
                    }
                } else if (Math.abs(setPosition - getPosition()) <= 1.0 / DOWN_ACCEL) {
                    setPercentOutput(getPercentOutput() - DOWN_ACCEL * getPositionDiff());
                    if (getPercentOutput() >= 0.0) {
                        setPercentOutput(0.0);
                        finished = true;
                    }
                }
            }
    }

    /**
     * Sets the elevator to the entered position
     *
     * @param position desired elevator position
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
        distance = setPosition - getPosition();
        goingUp = distance > 0;
        atMaxSpeed = false;
        finished = false;
    }

    /**
     * moves the elevator at a speed (percent output)
     *
     * @param speed: speed of the elevator (-1.0 to 1.0)
     */
    public static void setPercentOutput(double speed) {
        elevator.set(speed);
    }

    // /**
    // * Prevents the user from going past the max/min value of the elevator
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
    // * Prevents the user from going past the maximum value of the elevator
    // */
    // private static void limitPositionOverride() {
    // setPosition = Math.min(setPosition, MAX_POSITION + zeroPosition);
    // elevator.getPIDController().setReference(setPosition,
    // ControlType.kSmartMotion, 0, gravityFF);
    // }

    /**
     * @return the position of the elevator
     */
    public static double getPosition() {
        return elevatorEncoder.getPosition()/* - zeroPosition */; // removed to try to fix problem switch back if manual
                                                                  // override does a bad
    }

    /**
     * @return difference in encoder position since last check
     */
    public static double getPositionDiff() {
        double currentPosition = getPosition();
        double positionDiff = currentPosition - prevPosition;
        prevPosition = currentPosition;
        return positionDiff;
    }

    /**
     * @return true if the elevator is within deadband of its set value
     */
    public static boolean atSetPosition() {
        return Math.abs(elevatorEncoder.getPosition() - setPosition) < DEADBAND;
    }

    /**
     * sets the elevator set value to its current value
     */
    public static void stop() {
        setPosition(getPosition()); // if this is bad change it back
        // setPosition = elevatorEncoder.getPosition();
        // setPosition(setPosition);
    }

    /**
     * sets the elevator set value to its current value
     */
    public static double getCurrent() {
        return elevator.getOutputCurrent();
    }

    /**
     * @return the set point of the elevator
     */
    public static double getSetPosition() {
        return setPosition - zeroPosition;
    }

    /**
     * @return true if the elevator is above the stationary stage
     */
    public static boolean aboveStageThreshold() {
        return elevatorEncoder.getPosition() > STAGE_THRESHOLD + zeroPosition;
    }

    /**
     * Sets the current elevator position to the new zero
     */
    public static void resetEncoder() {
        // elevator.setEncPosition(0);
        zeroPosition = elevatorEncoder.getPosition();
        setPosition = Position.ELEVATOR_FLOOR.value;
        setPosition(setPosition);
    }

    /**
     * @return belt speed in inches per second
     */
    public static double getBeltVelocity() {
        final double kNeoToBelt = 16.213 / 60 /* min to sec */ / 25.4 /* mm to inch */;
        return kNeoToBelt * elevatorEncoder.getVelocity();
    }

    // /**
    // * Zeros the elevator if Hal Sensor is triggered, must be ran in robotPeriodic
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