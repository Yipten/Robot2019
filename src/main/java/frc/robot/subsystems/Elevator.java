package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {
    public static final CANSparkMax elevator;
    public static final CANEncoder elevatorEncoder;
    public static final DigitalInput halSensor;

    private static double setPosition, prevPosition, zeroPosition;
    private static double prevVelocity, currentVelocity;
    private static double currentAcceleration;
    public static final double gravityFF = 0.05 * 12; // .375

    private static final double STAGE_THRESHOLD = 30.0;
    public static final double MAX_POSITION = 47.5;
    private static final double DEADBAND = 0.26;
//    private static final double HATCH_DROP_OFFSET = 3.2;
//    private static final double HATCH_PLACE_OFFSET = 1.5; //1.2

//    private static final double highAccel = 30000;
//    private static final double mediumAccel = 10000; // was 13000
//    private static final double slowMediumAccel = 6000; // was 11000
//    private static final double lowAccel = 3000; // was 7000

    private static final double avagadrosVelocity = 6.02E23;
    private static boolean goingDown;

    public static boolean setHatchDrop, setHatchPlace, overriding; //TODO do we actually use all of these?

    /**
     * Initializes the elevator motor, sets PID values, and zeros the elevator
     * encoder
     */
    static {
        elevator = new CANSparkMax(5, MotorType.kBrushless);
        elevator.setInverted(false);
        elevator.getPIDController().setOutputRange(-1.0, 1.0);
        elevator.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        elevator.getPIDController().setSmartMotionMaxAccel(30000, 0);
        elevator.getPIDController().setSmartMotionMaxVelocity(5000, 0); // avagadro
        elevator.getPIDController().setSmartMotionAllowedClosedLoopError(0.2, 0);
        elevator.getPIDController().setSmartMotionMinOutputVelocity(0.01, 0);
        elevator.setIdleMode(CANSparkMax.IdleMode.kCoast);
//        elevator.enableVoltageCompensation(13.00); //TODO Sydney made this 13 (not 12) because it is slower when charged and I think this is why? - battery always starts above 12v

        elevator.getPIDController().setReference(0, ControlType.kSmartMotion, 0, 0);

        elevator.setSmartCurrentLimit(60);

        elevator.getPIDController().setP(0.000_4); // was .000_05 // 0.000_11
        elevator.getPIDController().setI(0.0);   // was 5.0E-9
        elevator.getPIDController().setIZone(2);
        elevator.getPIDController().setD(0.0); // was 0.003
        elevator.getPIDController().setFF(0.0);

        elevatorEncoder = new CANEncoder(elevator);
        halSensor = new DigitalInput(3);

        elevator.setEncPosition(0);
        setPosition = elevatorEncoder.getPosition();
        prevPosition = elevatorEncoder.getPosition();
        zeroPosition = elevatorEncoder.getPosition();
        prevVelocity = 0;

        setHatchDrop = false;
        setHatchPlace = false;
        overriding = false;
    }

    /**
     * Sets the elevator to the entered value
     *
     * @param targetPosition: desired elevator value
     */
    public static void setPosition(double targetPosition) {
        if (targetPosition <= getPosition()) { // down
            goingDown = true;
            if (getPosition() - targetPosition < 25)  // down medium
                if (getPosition() - targetPosition < 4)  // down short
                    elevator.getPIDController().setSmartMotionMaxAccel(7_500, 0);
                else
                    elevator.getPIDController().setSmartMotionMaxAccel(7_500, 0);
            else  // down big
                elevator.getPIDController().setSmartMotionMaxAccel(6_500, 0);
        } else { // up
            goingDown = false;
            if (targetPosition - getPosition() < 25)  // up medium
                elevator.getPIDController().setSmartMotionMaxAccel(16_000, 0); // 400_000, 0);
            else  // up high
                elevator.getPIDController().setSmartMotionMaxAccel(16_000, 0); // 400_000, 0);
        }

        if (!setHatchPlace) { //TODO this makes it so that placing works after the elevator zeros - Sydney (I'm an idiot)
            setPosition = targetPosition + zeroPosition;
        } else {
            setPosition = targetPosition;
        }

        if (!overriding) {
            limitPosition();
        } else {
            limitPositionOverride();
        }
        setHatchDrop = false;
        setHatchPlace = false;
    }

    /**
     * @return the position of the elevator
     */
    public static double getPosition() {
        return elevatorEncoder.getPosition()/* - zeroPosition*/; // removed to try to fix problem switch back if manual override does a bad
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
//        setPosition = elevatorEncoder.getPosition();
//        setPosition(setPosition);
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
     * Prevents the user from going past the maximum value of the elevator
     */
    private static void limitPositionOverride() {
        setPosition = Math.min(setPosition, MAX_POSITION + zeroPosition);
        elevator.getPIDController().setReference(setPosition, ControlType.kSmartMotion, 0, gravityFF);
    }

    /**
     * Sets the elevator to the entered position
     *
     * @param position desired elevator position
     */
    public static void setPosition(Position position) {
//        if (position.name().toLowerCase().contains("hatch")) {
//            setPosition(position.value + HATCH_DROP_OFFSET);
//        } else {
        setPosition(position.value);
//        }
    }

    /**
     * @return true if the elevator is above the stationary stage
     */
    public static boolean aboveStageThreshold() {
        return elevatorEncoder.getPosition() > STAGE_THRESHOLD + zeroPosition;
    }

    /**
     * moves the elevator at a speed (percent output)
     *
     * @param speed: speed of the elevator (-1.0 to 1.0)
     */
    public static void moveSpeed(double speed) {
//        elevator.set(speed);
        setPosition += (0.7 * speed);
        overriding = true;
        setHatchPlace = true;
        setPosition(setPosition);
    }

    /**
     * Prevents the user from going past the maximum value of the elevator
     */
    private static void limitPosition() {
        setPosition = Math.min(setPosition, MAX_POSITION + zeroPosition);
        setPosition = Math.max(setPosition, zeroPosition);
        if (goingDown) {
            elevator.getPIDController().setReference(setPosition, ControlType.kSmartMotion);
        } else {
            elevator.getPIDController().setReference(setPosition, ControlType.kSmartMotion, 0, gravityFF * 2);
        }
    }

    /**
     * Sets the current elevator position to the new zero
     */
    public static void resetEncoder() {
//        elevator.setEncPosition(0);
        zeroPosition = elevatorEncoder.getPosition();
        setPosition = Position.ELEVATOR_FLOOR.value;
        setPosition(setPosition);
    }

    /**
     * @return belt speed in inches per second
     */
    public static double getBeltVelocity() {
        final double kNeoToBelt = 16.213 / 60 /*min to sec*/ / 25.4 /*mm to inch*/;
        return kNeoToBelt * elevatorEncoder.getVelocity();
    }

    /**
     * Zeros the elevator if Hal Sensor is triggered, must be ran in robotPeriodic
     */
    public static void checkHalSensor() {
        if (!halSensor.get()) {
            zeroPosition = elevatorEncoder.getPosition() - 8.6; //9.6
            setPosition = setPosition + zeroPosition;
            limitPosition();
        }
    }

    public static double getVelocity() {
        prevVelocity = currentVelocity;
        currentVelocity = elevatorEncoder.getVelocity();
        return currentVelocity;
    }

    public static double getMotorOutput() {
        return elevator.getAppliedOutput();
    }

    /**
     * Positions of elevator
     */
    public enum Position {
        ELEVATOR_FLOOR(0.0),
        HATCH_LEVEL_ONE(0.0),
        HATCH_LEVEL_TWO(23.5),
        HATCH_LEVEL_THREE(45.3),
        CARGO_LEVEL_ONE(17.0),
        CARGO_LEVEL_TWO(39.0),
        CARGO_LEVEL_THREE(47.0),
        CARGO_SHIP(33.0),
        ELEVATOR_COLLECT_CARGO(7.7), //7.7
        //        ELEVATOR_COLLECT_HATCH(HATCH_DROP_OFFSET + HATCH_PLACE_OFFSET);
        ELEVATOR_COLLECT_HATCH(0.5); //???Do we need this? TODO possibly remove?
        public double value;

        Position(double position) {
            value = position;
        }
    }

    public static double getAcceleration() {
        currentAcceleration = (currentVelocity - prevVelocity) / .02;
        return currentAcceleration;
    }

    public static double getTemperature() {
        return elevator.getMotorTemperature();
    }

    public static double percentOutput() {
        return elevator.getAppliedOutput();
    }
}