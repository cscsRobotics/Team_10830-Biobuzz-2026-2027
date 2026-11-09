package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class TankDrive {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;


    public void init(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FLMoto");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BLMoto");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FRMoto");
        backRightDrive = hardwareMap.get(DcMotor.class, "BRMoto");

        //odometry computer configuration

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void tankDrive(double leftPower, double rightPower) {
        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);

        frontRightDrive.setPower(rightPower);
        backRightDrive.setPower(rightPower);
    }
    public void stop() {
        tankDrive(0, 0);
    }

}