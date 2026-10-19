package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class FieldOrientatedDrive {

    GoBildaPinpointDriver pinpoint;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    

    public void init(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "FLMoto");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BLMoto");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FRMoto");
        backRightDrive = hardwareMap.get(DcMotor.class, "BRMoto");

        //odometry computer configuration
        pinpoint.setOffsets(-185.0,-133.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //odometry starting position
        pinpoint.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        //fix starting position for pinpoint for TeleOp and Autonomous modes to ensure accurate readings of the computer
        pinpoint.setPosition(startingPosition);



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

        double maxPower = 1.0;
        double maxSpeed = 1.0;

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

        pinpoint.update();

        Pose2D pos = pinpoint.getPosition();

        double botHeading = pos.getHeading(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading)
                - forward * Math.sin(-botHeading);

        double rotY = strafe * Math.sin(-botHeading)
                + forward * Math.cos(-botHeading);

        drive(rotY, rotX, rotate);
    }
    public double getHeadingDegrees() {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        return pos.getHeading(AngleUnit.DEGREES);
    }

}