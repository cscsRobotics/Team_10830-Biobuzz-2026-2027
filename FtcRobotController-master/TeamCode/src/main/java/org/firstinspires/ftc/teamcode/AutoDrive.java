package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.regex.Pattern;  // utility to check strings
import java.util.logging.Level;  // log app messages
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.Map;
import java.util.Locale;

import java.lang.annotation.Target;

import java.io.FileOutputStream;
import java.io.File;
import java.io.IOException;


// Telemetry - print on driver hub
//
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.io.FileOutputStream; // ?
import java.io.File;             // ?
import java.io.IOException;      // ?


// Robot Hardware Classes
//
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

//
// LimeLight classes
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

// Rev LED Blinkin Classes
//
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

// Driving with Gyro (IMU) classes
//
// import for IMU (gyroscope)
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

// Not sure if needed.
//
import com.qualcomm.robotcore.util.RobotLog;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
public class AutoDrive {

    GoBildaPinpointDriver pinpoint;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private DcMotor intakeMoto;

    private double targetHeading;
    private final LinearOpMode opMode;
    private VoltageSensor voltageSensor;

    private static final double batteryConst = 13.5;

    private static final Double wheelCircumference = 4.0 * Math.PI;
    private static final Double gearRatio = 18.9;                  // Rev HD Hex 20:1
    private static final Double countsPerRotation = 560.0;         // Rev HD Hex 20:1
    private static final Double scaleFactor = 17.0;                 // need to find scale factor!!!
    private static final Double strafeFactor = 1.0;
    private static final Double countsPerInch = countsPerRotation / wheelCircumference / gearRatio * scaleFactor;


    public AutoDrive(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hardwareMap) {


        voltageSensor = hardwareMap.voltageSensor.iterator().next();

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

        pinpoint.update();

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

    public double getHeadingDegrees() {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        return pos.getHeading(AngleUnit.DEGREES);
    }

    public void resetEncoders()
    {
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders()
    {
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDrivePower(double FLpower,double FRpower, Double BLpower, Double BRpower)
    {
        frontRightDrive.setPower(FRpower);
        backRightDrive.setPower(BRpower);
        frontLeftDrive.setPower(FLpower);
        backLeftDrive.setPower(BLpower);
    }

    public double checkDirection() {
        double currentHeading = getHeadingDegrees();

        double error = normalizeHeading(targetHeading - currentHeading);

        double Kp = 0.015;

        return error * Kp;
    }
    /************************************************************/
    /* Function: strafeRight                                    */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move sideways  */
    /* in a right direction                                     */
    /************************************************************/
    public void strafeRight(Double inches,Double power)
    {
        driveInches(inches*strafeFactor, power, -power, -power, power);

    }

    /************************************************************/
    /* Function: strafeLeft                                     */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move sideways  */
    /* in a left  direction                                     */
    /************************************************************/
    public void strafeLeft(Double inches,Double power)
    {
        driveInches(inches*strafeFactor, -power, power, power, -power);
    }

    /************************************************************/
    /* Function: driveForward                                   */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move forward.  */
    /* The robot speed is passed in and that value is used for  */
    /* all wheels.                                              */
    /************************************************************/
    public void driveForward(Double inches,Double power)
    {
        driveInches(inches, power, power, power, power);
    }

    /***************************************************************/
    /* Function: driveBackward                                     */
    /* Returns: nothing                                            */
    /*                                                             */
    /* This function is called to have the robot move in reverse.  */
    /* The robot speed is passed in and that value is used for     */
    /* all wheels.                                                 */
    /***************************************************************/
    public void driveBackward(Double inches,Double power)
    {
        driveInches(inches, -power, -power, -power, -power);
    }

    public double normalizeHeading(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle <= -180.0) {
            angle += 360.0;
        }
        return angle;
    }
    /************************************************************/
    /*                                                          */
    /* Function: driveInches                                    */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move straigh   */
    /* in a forward or reverse direction.                       */
    /*                                                          */
    /* Strafe Forward = negative front wheels, positive back    */
    /* wheels                                                   */
    /************************************************************/
    public void driveInches(Double inches,Double FLpower,Double FRpower, Double BLpower, Double BRpower) {

            double counts = Math.abs(inches * countsPerInch);

            resetEncoders();
            runUsingEncoders();

        targetHeading = getHeadingDegrees();

            double voltage = voltageSensor.getVoltage();

            // Prevent division by zero or invalid voltage readings.
            if (voltage <= 0) {
                setDrivePower(0.0, 0.0, 0.0, 0.0);
                return;
            }

        FLpower = Range.clip(batteryConst * FLpower / voltage, -1.0, 1.0);
        FRpower = Range.clip(batteryConst * FRpower / voltage, -1.0, 1.0);
        BLpower = Range.clip(batteryConst * BLpower / voltage, -1.0, 1.0);
        BRpower = Range.clip(batteryConst * BRpower / voltage, -1.0, 1.0);

            ElapsedTime driveTimer = new ElapsedTime();

            // Safety timeout: stop if the requested movement takes too long.
            double timeoutSeconds = 5.0 + Math.abs(inches) / 5.0;

            while (opMode.opModeIsActive()
                    && driveTimer.seconds() < timeoutSeconds) {

                pinpoint.update();

                boolean FLdone =
                        Math.abs(frontLeftDrive.getCurrentPosition()) >= counts;

                boolean FRdone =
                        Math.abs(frontRightDrive.getCurrentPosition()) >= counts;

                boolean BLdone =
                        Math.abs(backLeftDrive.getCurrentPosition()) >= counts;

                boolean BRdone =
                        Math.abs(backRightDrive.getCurrentPosition()) >= counts;

                // Finish once every wheel reaches its target.
                if (FLdone && FRdone && BLdone && BRdone) {
                    break;
                }

                double correction = checkDirection();

                setDrivePower(
                        Range.clip(FLpower - correction, -1.0, 1.0),
                        Range.clip(FRpower + correction, -1.0, 1.0),
                        Range.clip(BLpower - correction, -1.0, 1.0),
                        Range.clip(BRpower + correction, -1.0, 1.0)
                );

                opMode.idle();
            }

            // Always stop the drive motors when the loop ends.
            setDrivePower(0.0, 0.0, 0.0, 0.0);

            runUsingEncoders(); // Stop all motors



        }


        //***************************************************************************
        // Functions for turning and checking robot angle for correction
        //

        /***************************************************/
        /* Resets the cumulative angle tracking to zero.   */
        /***************************************************/


        /************************************************************/
        /* Get current cumulative angle rotation from last reset.   */
        /* @return Angle in degrees. + = left, - = right.           */
        /************************************************************/

        /**
         * See if we are moving in a straight line and if not return a power correction value.
         * @return Power adjustment, + is adjust left - is adjust right.
         */

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpoint.update();

        double targetHeading = normalizeHeading(getHeadingDegrees() + degrees
        );

        ElapsedTime rotateTimer = new ElapsedTime();
        double timeoutSeconds = 5.0;

        while (opMode.opModeIsActive()
                && rotateTimer.seconds() < timeoutSeconds) {

            pinpoint.update();

            double currentHeading = normalizeHeading(
                    getHeadingDegrees()
            );

            double error = normalizeHeading(
                    targetHeading - currentHeading
            );

            // Stop when the robot is within 2 degrees of the target.
            if (Math.abs(error) <= 2.0) {
                break;
            }

            if (error > 0) {
                // Positive error: turn left.
                setDrivePower(-power, power, -power, power);
            } else {
                // Negative error: turn right.
                setDrivePower(power, -power, power, -power);
            }

            opMode.telemetry.addData("Target Heading", "%.1f", targetHeading);
            opMode.telemetry.addData("Current Heading", "%.1f", currentHeading);
            opMode.telemetry.addData("Heading Error", "%.1f", error);
            opMode.telemetry.update();

            opMode.idle();
        }

        setDrivePower(0.0, 0.0, 0.0, 0.0);

        runUsingEncoders();

        opMode.sleep(250);
    }

}


