package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@Autonomous(name = "AutoMain", group = "")
public class AutoMain extends LinearOpMode {

    private PinpointOdometry pinpoint;

    // Drive Wheels
    // Create Drive Wheel motors
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // Create intake motor
    private DcMotor intakeMoto;

    // This declares the IMU needed to get the current direction the robot is facing

    private VoltageSensor VoltSens;

    /***************************************/
    /* Variables for driving with Encoders */
    /***************************************/
    private static final Double wheelCircumference = 4.0 * Math.PI;
    private static final Double gearRatio = 18.9;                  // Rev HD Hex 20:1
    private static final Double countsPerRotation = 560.0;         // Rev HD Hex 20:1
    private static final Double scaleFactor = 17.0;                 // need to find scale factor!!!
    private static final Double strafeFactor = 1.0;
    private static final Double countsPerInch = countsPerRotation / wheelCircumference / gearRatio * scaleFactor;

    // Global Variables
    private static Double noPower = 0.0;
    private static Double quarterPower = 0.25;
    private static Double oneThirdPower = 0.34;
    private static Double halfPower = 0.5;
    private static Double threeQuartPower = 0.75;
    private static Double fullPower = 1.0;

    /*********************/
    /* Variables for imu */
    /*********************/
    double               globalAngle, correction;
    YawPitchRollAngles   lastAngles;

    double voltage = 0;
    double batteryConst = 13.5;

    /*******************************************************************************************/
    /* Function: SelectDelayTime                                                               */
    /* Returns: Delay Time in milliseconds                                                     */
    /*******************************************************************************************/
    private Integer SelectDelayTime()
    {
        Integer delayTimeMilliseconds = 0;    // Initialize delay to be 0 seconds

        telemetry.addData("Delay","%d (Not Set)",delayTimeMilliseconds);
        telemetry.addData("Left Bumper -"," decrease delay time by (1 second)");
        telemetry.addData("Right bumper -", " increase delay time by (1 second)");
        telemetry.addData("a", "No delay time is needed or finished selecting time");
        telemetry.update();

        while (!isStopRequested() && gamepad1.a == false)
        {
            if (gamepad1.left_bumper)
            {
                delayTimeMilliseconds -= 1000;
                if (delayTimeMilliseconds < 0)
                {
                    delayTimeMilliseconds = 0;
                }
                while (gamepad1.left_bumper)
                {
                    idle();
                }
                telemetry.addData("Delay","%d (decrease)",delayTimeMilliseconds);
                telemetry.update();
            }

            if (gamepad1.right_bumper)
            {
                delayTimeMilliseconds += 1000;
                if (delayTimeMilliseconds > 10000)
                {
                    delayTimeMilliseconds = 10000;
                }
                while (gamepad1.right_bumper)
                {
                    idle();
                }
                telemetry.addData("Delay","%d (increase)",delayTimeMilliseconds);
                telemetry.update();
            }
        }

        while (!isStopRequested() && gamepad1.a)
        {
            idle();
        }

        telemetry.addData("Delay Time SET", delayTimeMilliseconds);
        telemetry.update();
        return delayTimeMilliseconds;
    }

    /******************************/
    /* OpMode for autonomous code */
    /******************************/
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Pinpoint Odometry
        pinpoint = new PinpointOdometry(hardwareMap);
        pinpoint.setPose(0.0, 0.0, 0.0);

        telemetry.addLine("Pinpoint Odometry Initialized!");
        telemetry.addLine("Keep robot still until start.");
        telemetry.update();

        // Mapped drive train wheels
        frontLeftDrive = hardwareMap.get(DcMotor.class,"FLMoto");
        frontRightDrive = hardwareMap.get(DcMotor.class,"FRMoto");
        backLeftDrive = hardwareMap.get(DcMotor.class,"BLMoto");
        backRightDrive = hardwareMap.get(DcMotor.class,"BRMoto");

        // Map intake motor
        intakeMoto = hardwareMap.get(DcMotor.class,"intake");

        VoltSens = hardwareMap.voltageSensor.get("Control Hub");

        //***************************************************//
        // Set direction of all motors                       //
        //***************************************************//
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

        intakeMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Integer delayTimeMilliseconds = 0;

        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        delayTimeMilliseconds = SelectDelayTime();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("mode","waiting for start");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.addData("Mode", "running");
        telemetry.update();

        resetEncoders();

        if (delayTimeMilliseconds > 0) {
            telemetry.addData("Status","Delaying");
            telemetry.update();

            sleep(delayTimeMilliseconds);

            telemetry.addData("Status","Running");
            telemetry.update();
        }

        telemetry.clearAll();

        // Autonomous Execution Flow
        if (opModeIsActive()) {
            driveForward(24.0, halfPower);
            sleep(250);

            // Intake execution example
            intakeMoto.setPower(fullPower);
            sleep(1000);
            intakeMoto.setPower(noPower);

            strafeRight(12.0, halfPower);
            sleep(250);

            driveBackward(12.0, halfPower);
            sleep(250);
        }
    }

    //**************************************************************
    // Functions needed for driving auto
    //

    private void resetEncoders()
    {
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runUsingEncoders()
    {
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDrivePower(double FLpower, double FRpower, Double BLpower, Double BRpower)
    {
        frontRightDrive.setPower(FRpower);
        backRightDrive.setPower(BRpower);
        frontLeftDrive.setPower(FLpower);
        backLeftDrive.setPower(BLpower);
    }

    private void strafeRight(Double inches, Double power)
    {
        driveInches(inches * strafeFactor, power, -power, -power, power);
    }

    private void strafeLeft(Double inches, Double power)
    {
        driveInches(inches * strafeFactor, -power, power, power, -power);
    }

    private void driveForward(Double inches, Double power)
    {
        driveInches(inches, power, power, power, power);
    }

    private void driveBackward(Double inches, Double power)
    {
        driveInches(inches, -power, -power, -power, -power);
    }

    private void driveInches(Double inches, Double FLpower, Double FRpower, Double BLpower, Double BRpower)
    {
        Double counts = inches * countsPerInch;

        resetEncoders();
        runUsingEncoders();

        voltage = VoltSens.getVoltage();

        FLpower = ((batteryConst * FLpower) / voltage);
        FRpower = ((batteryConst * FRpower) / voltage);
        BLpower = ((batteryConst * BLpower) / voltage);
        BRpower = ((batteryConst * BRpower) / voltage);

        setDrivePower(FLpower, FRpower, BLpower, BRpower);

        while (opModeIsActive() &&
                ((Math.abs(frontLeftDrive.getCurrentPosition()) + Math.abs(frontRightDrive.getCurrentPosition())) / 2) < Math.abs(counts))
        {
            correction = checkDirection();
            setDrivePower(FLpower - correction, FRpower + correction, BLpower - correction, BRpower + correction);
            idle();
        }
        setDrivePower(noPower, noPower, noPower, noPower);
    }

    /**
     * Bypasses internal IMU drift errors.
     * Computes drift error offsets by reading current heading directly from Pinpoint.
     */
    private double checkDirection()
    {
// Read directly from your custom Pinpoint wrapper method instead of old Control Hub imu loop
        double currentHeading = pinpoint.getHeadingDegrees();
        double error = -currentHeading;
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        double Kp = 0.015; // Proportional feedback modifier constant
        return error * Kp;
    }
    /**
     * Executes pure point turns utilizing your new lightning-fast hardware IMU heading.
     */
    private void rotate(int degrees, double power)
    {
// Change runmode to completely override encoder restrictions during a spin
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pinpoint.update();
// Use the absolute value heading from Pinpoint to define your exact target
        double targetHeading = pinpoint.getHeadingDegrees() + degrees;
// Keep loop active until absolute difference error is within a tight 2 degree window
        while (opModeIsActive() && Math.abs(pinpoint.getHeadingDegrees() - targetHeading) > 2.0) {
            pinpoint.update(); // Fetch fresh orientation parameters
            if (pinpoint.getHeadingDegrees() < targetHeading) {
// Spin Left (Counter-Clockwise)
                setDrivePower(-power, power, -power, power);
            } else {
// Spin Right (Clockwise)
                setDrivePower(power, -power, power, -power);
            }
            telemetry.addData("Target Heading", "%.1f", targetHeading);
            telemetry.addData("Current Pinpoint Heading", "%.1f", pinpoint.getHeadingDegrees());
            telemetry.update();
        }
// Clean halt safety distribution matrix sequence
        setDrivePower(noPower, noPower, noPower, noPower);
        sleep(250);
    }
}