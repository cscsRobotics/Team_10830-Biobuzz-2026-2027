package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.gamepad.Gamepad;

// Blinkin


// Driving with Gyro (IMU) classes
//
// import for IMU (gyroscope)


public class MotorsClass {

    private DcMotor intakeMoto;
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
        intakeMoto = hardwareMap.get(DcMotor.class,"intake");
       
        intakeMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //add more when needed

    }
     public void handleIntake(Gamepad gamepad) {
        if (gamepad.left_bumper) {
            intakeMotor.setPower(1.0);
        
        } else if (gamepad.right_bumper) {
            intakeMotor.setPower(-1.0);
           
        } else {
            intakeMotor.setPower(0.0);
        }
    }
}
