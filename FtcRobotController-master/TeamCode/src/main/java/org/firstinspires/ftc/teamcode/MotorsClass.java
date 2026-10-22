package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        sorterMoto = hardwareMap.get(DcMotor.class,"Sorter");


        intakeMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sorterMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        sorterMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // sorterMoto.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorterMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorterMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMoto.setTargetPosition(0);


    }


}
