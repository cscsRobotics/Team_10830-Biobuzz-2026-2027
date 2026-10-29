package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

// Blinkin


// Driving with Gyro (IMU) classes
//
// import for IMU (gyroscope)


public class MotorsClass {

    private DcMotor intakeMotor;
    private DcMotor sorterMoto;

    //create motor speeds
    private static double sorterPowerSpeed = 0.4;
    private static double sorterPowerSpeedIntake = 0.35;
    private static double sorterPowerSpeedReset = 1.0;
    private static int rotateSorterRotateDelay = 200;
    private static double sorterPowerSpeedStop = 0.0;

    private static int sorterIntakePos = 0;

    public void init(HardwareMap hardwareMap) {

        // map motors for ball intake and ball sorter
        intakeMotor = hardwareMap.get(DcMotor.class,"intake");
       
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
}
