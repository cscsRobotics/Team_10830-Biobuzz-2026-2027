package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class FieldOrientatedDrive {

    private PinpointOdometry odometry;
    private ControlHubIMU backupIMU;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;


    public void init(HardwareMap hardwareMap) {

        odometry = new PinpointOdometry(hardwareMap);
        backupIMU = new ControlHubIMU(
                hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        frontLeftDrive = hardwareMap.get(DcMotor.class, "FLMoto");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BLMoto");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FRMoto");
        backRightDrive = hardwareMap.get(DcMotor.class, "BRMoto");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void drive(double forward, double strafe, double rotate) {

        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxSpeed = 1.0;
        double maxPower = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));


        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {

        odometry.update();

        double botHeading = odometry.getHeading();

        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        drive(rotY, rotX, rotate);
    }

    public double getSafeHeadingRadians() {
        try {
            odometry.update();
            double headingDeg = odometry.getHeadingDegrees();

            // If Pinpoint returns NaN (disconnected/glitched), use backup IMU
            if (Double.isNaN(headingDeg)) {
                return backupIMU.getHeadingRadians();
            }
            return odometry.getHeading(); // Returns radians
        } catch (Exception e) {
            // Fallback to Control Hub IMU on exception
            return backupIMU.getHeadingRadians();
        }
    }
    public PinpointOdometry getOdometry() {
        return odometry;
    }

    public ControlHubIMU getBackupIMU() {
        return backupIMU;
    }

    // Add this method inside FieldOrientatedDrive.java
    public void resetHeading() {
        try {
            odometry.setPose(odometry.getXInches(), odometry.getYInches(), 0);
        } catch (Exception e) {
            // Ignore if Pinpoint is dead
        }
        backupIMU.resetYaw();
    }
}