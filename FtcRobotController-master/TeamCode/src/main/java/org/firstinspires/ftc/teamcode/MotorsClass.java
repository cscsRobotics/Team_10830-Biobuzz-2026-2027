package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;


public class MotorsClass {

    private DcMotor intakeMotor;
    // Create motors and velocity controller for flywheels
    private DcMotorEx flyWheelMotor;

    private DcMotor pusherUpper;

    private boolean previousGamepad1a = false;

    private boolean motorRunning = false;



    public void init(HardwareMap hardwareMap) {

        // map motors for ball intake and fly wheels
        intakeMotor = hardwareMap.get(DcMotor.class,"intake");

        flyWheelMotor = hardwareMap.get(DcMotorEx.class,"flyWheelMotor");

        pusherUpper = hardwareMap.get(DcMotor.class, "pusherUpper");


        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pusherUpper.setDirection(DcMotorSimple.Direction.FORWARD);
        pusherUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pusherUpper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pusherUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //add more when needed

    }
    public void handleIntake(Gamepad gamepad) {
        if (gamepad.right_bumper && gamepad.left_bumper) {
            intakeMotor.setPower(0.0);

        } else if (gamepad.right_bumper) {
            intakeMotor.setPower(-0.65);

        } else if (gamepad.left_bumper){
            intakeMotor.setPower(0.65);

        } else {
            intakeMotor.setPower(0.0);
        }
    }


    public void pusherUpperMech(Gamepad gamepad) {
        //check if gamepad.y is pressed
        if (gamepad.y) {
            pusherUpper.setPower(1.0);
        } else if (gamepad.x){ //check if gamepad.x is pressed
            pusherUpper.setPower(-1.0);
        } else {//if none pressed then set power to 0
            pusherUpper.setPower(0.0);
        }


    }

    public void handleFlyWheel(Gamepad gamepad) {
        // 1. Detect if the button was JUST pressed down
        if (gamepad.a && !previousGamepad1a) {
            motorRunning = !motorRunning;
        }

        // 2. ALWAYS update this variable, out of the 'if' statement
        previousGamepad1a = gamepad.a;

        // 3. Set the motor to full power (1.0) or off (0.0)
        if (motorRunning) {
            flyWheelMotor.setPower(1.0); // 1.0 means 100% power
        } else {
            flyWheelMotor.setPower(0.0);
        }
    }

}