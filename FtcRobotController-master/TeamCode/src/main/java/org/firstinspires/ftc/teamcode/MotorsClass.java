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


    public void init(HardwareMap hardwareMap) {

        // map motors for ball intake and fly wheels
        intakeMotor = hardwareMap.get(DcMotor.class,"intake");

        flyWheelMotor = hardwareMap.get(DcMotorEx.class,"flyWheelMotor");

       
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //add more when needed

    }
     public void handleIntake(Gamepad gamepad) {
        if (gamepad.right_bumper && gamepad.left_bumper) {
            intakeMotor.setPower(0.0);
        
        } else if (gamepad.right_bumper) {
            intakeMotor.setPower(-0.5);
           
        } else if (gamepad.left_bumper){
         intakeMotor.setPower(0.5);

        } else {
            intakeMotor.setPower(0.0);
        }
    }

    public void handleFlyWheel(Gamepad gamepad) {
        if (gamepad.a) {
            flyWheelMotor.setPower(0.85);
        } else {
            flyWheelMotor.setPower(0.0);
        }
        }

    }
