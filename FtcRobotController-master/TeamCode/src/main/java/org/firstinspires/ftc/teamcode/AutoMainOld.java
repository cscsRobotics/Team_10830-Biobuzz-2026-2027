package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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


@Autonomous(name = "AutoMain", group = "")

public class AutoMainOld extends LinearOpMode {

    private AutoDrive autoDrive;

    // Drive Wheels
    // Create Drive Wheel motors

    // Create motors and velocity controller for flywheels

    // private PIDFController velocityController;

    // Create other motors
    //
    private DcMotor intakeMoto;


    // Create all servos


    // This declares the IMU needed to get the current direction the robot is facing

    // Create color sensor
    private NormalizedColorSensor color;

    // Create Limelight
    private Limelight3A limeLight;

    /*
    private static int pipeLine21Red = 1;
    private static int pipeLine22Red = 2;
    private static int pipeLine23Red = 3;
    private static int pipeLine21Blue = 4;
    private static int pipeLine22Blue = 5;
    private static int pipeLine23Blue = 6;
    private static int pipeLine21 = 7;
    private static int pipeLine22 = 8;
    private static int pipeLine23 = 9; */

    private VoltageSensor VoltSens;

    //BlinkinLEDs
    private RevBlinkinLedDriver blinkinLedDriver;

    /***************************/
    /* Possible Automous Modes */
    /***************************/
    public enum Park
    {
        PARK_PATTERN_NOT_SELECTED,
        PARK_PATTERN_FRONTSTAGE,
        PARK_PATTERN_BACKSTAGE,
    }

    /***************************/
    /* Possible Park Location  */
    /***************************/
    public enum parkLocation
    {
        PARK_PATTERN_NOT_SELECTED,
        PARK_PATTERN_FRONTSTAGE,           // Apriltag #23
        PARK_PATTERN_BACKSTAGE,           // Apriltag #22
    }

    /**********************/
    /* Sorter Directions  */
    /**********************/

    /***************************************/
    /* Variables for driving with Encoders */
    /***************************************/
    private static final Double wheelCircumference = 4.0 * Math.PI;
    private static final Double gearRatio = 18.9;                  // Rev HD Hex 20:1
    private static final Double countsPerRotation = 560.0;         // Rev HD Hex 20:1
    private static final Double scaleFactor = 17.0;                 // need to find scale factor!!!
    private static final Double strafeFactor = 1.0;
    private static final Double countsPerInch = countsPerRotation / wheelCircumference / gearRatio * scaleFactor;


    // Gobal Variables
    private static Double noPower = 0.0;
    private static Double quarterPower = 0.25;
    private static Double oneThirdPower = 0.34;
    private static Double halfPower = 0.5;
    private static Double threeQuartPower = 0.75;
    private static Double fullPower = 1.0;
    private static int rotateOneBall = 765;
    private static int rotateOneBallCC = 745;
    private static int rotateTwoBalls = 720;
    private static double motorSpeedBlueGoal = 0.75;
    private static double motorSpeedBlueAud = 0.92;
    private static double motorSpeedRedGoal = 0.75;
    private static double motorSpeedRedAud = 0.92;

    // Blinkin LED Colors for Indication of detected Motifs
   // private RevBlinkinLedDriver.BlinkinPattern BasePattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    //private RevBlinkinLedDriver.BlinkinPattern PPGmotif = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    //private RevBlinkinLedDriver.BlinkinPattern PGPmotif = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    //private RevBlinkinLedDriver.BlinkinPattern GPPmotif = RevBlinkinLedDriver.BlinkinPattern.GREEN;


    /*********************/
    /* Variables for imu */
    /*********************/
    double               globalAngle, correction;
    YawPitchRollAngles   lastAngles;
    double voltage = 0;
    double batteryConst = 13.5;
    private static double powerConstant = .5;

    // Declare static variables
    private double reduceSpeed = 1.0;

    // set lift servo down and up positions
    // Larger number is down and small numbers raise lift higher


    //set pushBallFront servo down and up position
    // check for color ball - norm colors for green and red RBG

    //***********************************************************
    /* Calibration Numbers for color sensor values in classroom

         Note: need to be calibrated at each event!!!

         Green Ball:
              Red   Min: 0.40      Max: 0.45
              Green Min: 0.722     Max: 0.825
              Blue  Min: 0.211     Max: 0.41

         Purple Ball:
              Red   Min: 0.64      Max: 0.74
              Green Min: 0.54      Max: 0.58
              Blue  Min: 0.327     Max: 0.352

         No Ball:
              Red   Min: 0.143     Max: 0.148
              Green Min: 0.159     Max: 0.161
              Blue  Min: 0.086     Max: 0.087

    */
    //***********************************************************// greater than


    // Global Variables to  Game Specific Information
    Park  park = Park.PARK_PATTERN_NOT_SELECTED;         // store autonomous mode selected
    parkLocation parkLocal = parkLocation.PARK_PATTERN_NOT_SELECTED;  // store park location selected

    /*******************************************************************************************/
    /*

    /*******************************************************************************************/
    /* Function: SelectAutoMode                                                                */
    /* Returns: Selected mode                                                                  */
    /*                                                                                         */
    /* This function is use to select the automous code to be executed for this match          */
    /* Game pad 1 is used and the following buttons are used for selection:                    */
    /*      a - Red Goal                                                                       */
    /*      b - Red Audiance                                                                   */
    /*      x - Blue Goal                                                                      */
    /*      y - Blue Audiance                                                                  */
    /*******************************************************************************************/

    private Park SelectPark()
    {
        Park park = Park.PARK_PATTERN_NOT_SELECTED;      // Local variable to store selected automous mode

        /*******************************************/
        /* Display automous mode not selected yet  */
        /*******************************************/
        telemetry.addData("AutoMode","Not Selected");
        telemetry.addData("AutoMode","Select a - Park frontstage");
        telemetry.addData("AutoMode","Select b - Park Backstage");

        telemetry.update();

        /*****************************************/
        /* Loop until automous mode is selected  */
        /*****************************************/
        while (!isStopRequested() && park == park.PARK_PATTERN_NOT_SELECTED)  {

            if (gamepad1.a) {  // Red side away from audience/Blue side toward audience
                park = Park.PARK_PATTERN_FRONTSTAGE;
            }

            if (gamepad1.b) {  // Red side toward audience/Blue side away from audience
               park = Park.PARK_PATTERN_BACKSTAGE;
            }
            idle();
        }

        /************************************/
        /* Display selected automous mode   */
        /************************************/
        // mode.setValue(autoMode.toString());
        telemetry.addData("Automous Mode", park.toString());
        telemetry.update();

        // Wait for the user to release the button
        while (!isStopRequested() && (gamepad1.a || gamepad1.b)) {
            idle();
        }

        return park;

    } // end SelectAutoMode


    /*******************************************************************************************/
    /* Function: SelectDelayTime                                                               */
    /* Returns: Delay Time in milliseconds                                                     */
    /*                                                                                         */
    /* This function is use to select how long to delay the start of the automous code.        */
    /* Game pad 1 is used and the following controls are used for selection:                   */
    /*        left bumper - decrease delay time by 1000 milliseconds (1 second)                */
    /*        right bumper - increase delay time by 1000 milliseconds (1 second)               */
    /*        a button -    set selected time                                                  */
    /*                                                                                         */
    /*                    note: if no delay time is needed, just select the a button.     The  */
    /*                          default for the delay time is 0.                               */
    /*******************************************************************************************/
    private Integer SelectDelayTime()
    {
        Integer delayTimeMilliseconds = 0;    // Initialize delay to be 0 seconds

        // display delay time not set
        telemetry.addData("Delay","%d (Not Set)",delayTimeMilliseconds);
        telemetry.addData("Left Bumper -"," decrease delay time by (1 second)");
        telemetry.addData("Right bumper -", " increase delay time by (1 second)");
        telemetry.addData("a", "No delay time is needed or finished selecting time");
        telemetry.update();

        /*******************************************************************************/
        /* Select Delay time.                                                          */
        /*   - Select 'a' button without hitting bumpers if no delay needed            */
        /*   - Use Left Bumper to decrease delay time                                  */
        /*   - Use Right bumper to increase delay time                                 */
        /*                                                                             */
        /* Note:    After entering delay time, use "a" button to set selected time     */
        /*******************************************************************************/
        while (!isStopRequested() && gamepad1.a == false)
        {
            if (gamepad1.left_bumper)
            {
                delayTimeMilliseconds -= 1000;

                // ensure delay time does not go negetive
                if (delayTimeMilliseconds < 0)
                {
                    delayTimeMilliseconds = 0;
                }

                // Wait for the bumper to be released
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

                // ensure delay time is not greater than 10 seconds
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

        // Wait for user to release the a button
        while (!isStopRequested() && gamepad1.a)
        {
            idle();
        }

        /**************************************/
        /* Display selected delay time        */
        /**************************************/
        // delay.setValue("%d", delayTimeMilliseconds);
        telemetry.addData("Delay Time SET", delayTimeMilliseconds);
        telemetry.update();
        return delayTimeMilliseconds;       // returns selected delay time

    } // end SelectDelayTime

    /******************************/
    /* OpMode for automous code   */
    /******************************/
    @Override
    public void runOpMode() throws InterruptedException
    {

        autoDrive = new AutoDrive(this);
        autoDrive.init(hardwareMap);

        double tgtPower = 0;

        //****************************************//
        // Map all robot hardware                 //
        //****************************************//


        //map sensor
        //color = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        //color.setGain(10);  // increase sensitivity of the color sensor

        VoltSens = hardwareMap.voltageSensor.get("Control Hub");

        // Blinkin - map LED lights
        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
        //blinkinLedDriver.setPattern(BasePattern);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */

        telemetry.addData("Mode","calibrating imu...." );
        telemetry.update();


        // Map LimeLight
        //limeLight = hardwareMap.get(Limelight3A.class, "limelight");

        // Create local variable to store selected automous mode
        // and amount of delay time
        Integer delayTimeMilliseconds = 0;      // variable to store how long to delay before starting automous

        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        /*********************************************/
        /* Select Automous Mode, and Delay time      */
        /*********************************************/
        park = SelectPark();
        delayTimeMilliseconds = SelectDelayTime();

        // limeLight.start();

        /**********************************************************/
        /* All required data entered.  Automous is initialized.     */
        /**********************************************************/
        telemetry.addData("Status", "Initialized");
        telemetry.addData("mode","waiting for start");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        /**********************************/
        /* Wait for start of the match     */
        /**********************************/
        waitForStart();

        telemetry.clearAll();               // clear display messages

        telemetry.addData("Mode", "running");
        telemetry.update();

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         * The limeLight will drain the battery so since this is auto, we should start the limeLight after
         * the start button pressed.  If limeLight slow to start then will need to move start.
         */
        //limeLight.start();

        // Delay start if needed
        if (delayTimeMilliseconds > 0) {
            telemetry.addData("Status","Delaying");    // display delay status
            telemetry.update();

            sleep(delayTimeMilliseconds);             // wait selected amount of time

            telemetry.addData("Status","Running");  // dispay delay over and automous code is running
            telemetry.update();
        }

        telemetry.clearAll();// clear display messages

        autoDrive.driveForward(4.0, halfPower);

        switch (park) {
            case PARK_PATTERN_FRONTSTAGE:

                autoDrive.strafeRight(5.0, halfPower);

                break;

            case PARK_PATTERN_BACKSTAGE:

                autoDrive.strafeLeft(5.0, halfPower);

                break;

            case PARK_PATTERN_NOT_SELECTED:
                // This one should not happen if it does do nothing
                break;
        }


    }

/*
    private motifs getMotif() {

        boolean targetFound = false;

        // Set return motif pattern to default
        motifs motifPattern = motifs.MOTIF_PATTERN_NOT_SELECTED;

        switch (autoMode) {

            case AUTO_MODE_RED_GOAL:
            case AUTO_MODE_BLUE_GOAL:

                limeLight.pipelineSwitch(pipeLine21); // need to setup pipeline to check for 21
                sleep(100);

                targetFound  = checkTarget();

                if (targetFound) {
                    motifPattern =  motifs.MOTIF_PATTERN_GPP;
                    blinkinLedDriver.setPattern(GPPmotif);

                    break;
                }
                else
                {
                    limeLight.pipelineSwitch(pipeLine22); // need to setup pipeline to check for 22
                    sleep(100);

                    targetFound = checkTarget();

                    if (targetFound) {
                        motifPattern =  motifs.MOTIF_PATTERN_PGP;
                        blinkinLedDriver.setPattern(PGPmotif);

                        break;
                    }
                    else
                    {
                        limeLight.pipelineSwitch(pipeLine23); // need to setup pipeline to check for 23
                        sleep(100);

                        targetFound = checkTarget();

                        telemetry.addData("Target Found", targetFound);
                        telemetry.update();

                        if (targetFound) {
                            motifPattern =  motifs.MOTIF_PATTERN_PPG;
                            blinkinLedDriver.setPattern(PPGmotif);

                            break;
                        }
                    }
                }

                break;

            case AUTO_MODE_RED_AUDIENCE:

                limeLight.pipelineSwitch(pipeLine21Red); // need to setup pipeline to check for 21
                sleep(100);

                targetFound = checkTarget();

                if (targetFound) {
                    motifPattern =  motifs.MOTIF_PATTERN_GPP;
                    blinkinLedDriver.setPattern(GPPmotif);

                    break;
                }
                else
                {
                    limeLight.pipelineSwitch(pipeLine22Red); // need to setup pipeline to check for 22
                    sleep(100);

                    targetFound = checkTarget();

                    if (targetFound) {
                        motifPattern =  motifs.MOTIF_PATTERN_PGP;
                        blinkinLedDriver.setPattern(PGPmotif);

                        break;
                    }
                    else
                    {
                        limeLight.pipelineSwitch(pipeLine23Red); // need to setup pipeline to check for 23
                        sleep(100);

                        targetFound = checkTarget();

                        if (targetFound) {
                            motifPattern =  motifs.MOTIF_PATTERN_PPG;
                            blinkinLedDriver.setPattern(PPGmotif);

                            break;
                        }
                    }

                }

                break;

            case AUTO_MODE_BLUE_AUDIENCE:

                limeLight.pipelineSwitch(pipeLine21Blue); // need to setup pipeline to check for 21
                sleep(100);

                targetFound = checkTarget();

                if (targetFound) {
                    motifPattern =  motifs.MOTIF_PATTERN_GPP;
                    blinkinLedDriver.setPattern(GPPmotif);

                    break;
                }
                else
                {
                    limeLight.pipelineSwitch(pipeLine22Blue); // need to setup pipeline to check for 22
                    sleep(100);

                    targetFound = checkTarget();

                    if (targetFound) {
                        motifPattern =  motifs.MOTIF_PATTERN_PGP;
                        blinkinLedDriver.setPattern(PGPmotif);

                        break;
                    }
                    else
                    {
                        limeLight.pipelineSwitch(pipeLine23Blue); // need to setup pipeline to check for 23
                        sleep(100);

                        targetFound = checkTarget();

                        if (targetFound) {
                            motifPattern =  motifs.MOTIF_PATTERN_PPG;
                            blinkinLedDriver.setPattern(PPGmotif);

                            break;
                        }
                    }

                }

                break;

            case AUTO_MODE_NOT_SELECTED:
                // This one should not happen if it does do nothing
                break;
        }

        return motifPattern;
    }
*/



}

