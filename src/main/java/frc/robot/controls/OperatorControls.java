package frc.robot.controls;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

import static frc.robot.subsystems.Arm.Position.*;
import static frc.robot.subsystems.Elevator.Position.*;

public class OperatorControls {

    private static PS4Controller operatorJoy;
    private static boolean elevatorOverriden, armOverriden;

    /*
     * Initializes the operator controller
     */
    static {
        operatorJoy = new PS4Controller(1, 3);
        elevatorOverriden = false;
        armOverriden = false;
    }

    /**
     * Runs the operator controls
     */
    public static void operatorControls() {
        //Elevator Controls
          //override
        if (operatorJoy.getRightStickButton()) {
            double speedE = operatorJoy.getRightYAxis();
            Elevator.moveSpeed(-speedE);
            elevatorOverriden = true;
        } else {
            if(elevatorOverriden){
                Elevator.stop();
                elevatorOverriden = false;
            }
        }
         //positions
        if (operatorJoy.getLeftYAxis() > 0.1) {
            if (operatorJoy.getButtonDDown()) {
                Elevator.setPosition(CARGO_LEVEL_ONE);
            } else if (operatorJoy.getButtonDLeft()) {
                Elevator.setPosition(CARGO_LEVEL_TWO);
            } else if (operatorJoy.getButtonDUp()) {
                Elevator.setPosition(CARGO_LEVEL_THREE);
            }
        } else if (operatorJoy.getLeftYAxis() < -0.1) {
            if (operatorJoy.getButtonDDown()) {
                Elevator.setPosition(HATCH_LEVEL_ONE);
            } else if (operatorJoy.getButtonDLeft()) {
                Elevator.setPosition(HATCH_LEVEL_TWO);
            } else if (operatorJoy.getButtonDUp()) {
                Elevator.setPosition(HATCH_LEVEL_THREE);
            }
        }
          //encoder reset
        if(operatorJoy.getShareButton()){
            Elevator.resetEncoder();
        }

        //Arm Controls
          //override
        if(operatorJoy.getLeftStickButton()){
            double speedA = operatorJoy.getLeftYAxis();
            Arm.moveSpeed(speedA);
            armOverriden = true;
        } else {
            if(armOverriden){
                Arm.stop();
                armOverriden = false;
            }
        }
          //positions
        if (operatorJoy.getLeftYAxis() > 0.5) {
            Arm.setPosition(CARGO_SHOOT);
        } else if (operatorJoy.getLeftYAxis() < -0.5) {
            Arm.setPosition(ARM_FLOOR);
        } else if (Math.abs(operatorJoy.getLeftXAxis()) > 0.5) {
            Arm.setPosition(PLACING);
        } else if (operatorJoy.getSquareButton()) {
            Arm.setPosition(STARTING_CONFIG);
            Elevator.setPosition(ELEVATOR_FLOOR);
        }
          //encoder reset 
        if(operatorJoy.getOptionsButton()){
            Arm.resetEncoder();
        }

        //Intake Controls
          //cargo
        if (operatorJoy.getXButton()) {
            //Intake.intakeMode(); //TODO enable sensor - not plugged in for initial testing
            Intake.collectCargo();
        }
        if (operatorJoy.getTriButton()) {
            Intake.outtakeCargo();
        }
        if (operatorJoy.getCircleButton()) {
            Intake.stopCargoRollers();
        }
        if (operatorJoy.getRightTriggerAxis() > 0.5) {
            Intake.spitCargo();
        }
          //hatch
        if (operatorJoy.getLeftBumperButton()) {
            Intake.openHatch();
        }
        if (operatorJoy.getRightBumperButton()) {
            Intake.closeHatch();
        }

        //Combined Subsystem Controls
        if (operatorJoy.getLeftTriggerAxis() > 0.5) {
            Elevator.setPosition(ELEVATOR_FLOOR);
            Arm.setPosition(ARM_FLOOR);
        }

        //Controller Vibrations
        if(Intake.intakeRunning()){
            operatorJoy.setRumble(1.0);
        }
    }
}