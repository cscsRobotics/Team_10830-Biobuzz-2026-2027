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

    // Drive Wheels
    // Create Drive Wheel motors

    private PinpointOdometry pinpoint;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // Create motors and velocity controller for flywheels

    // private PIDFController velocityController;

    // Create other motors
    //
    private DcMotor intakeMoto;


    // Create all servos


    // This declares the IMU needed to get the current direction the robot is facing
    private Orientation lastAngle = new Orientation();

    // Create color sensor
    private NormalizedColorSensor color;

    // Create Limelight
    private Limelight3A limeLight;

    private static int pipeLine21Red = 1;
    private static int pipeLine22Red = 2;
    private static int pipeLine23Red = 3;
    private static int pipeLine21Blue = 4;
    private static int pipeLine22Blue = 5;
    private static int pipeLine23Blue = 6;
    private static int pipeLine21 = 7;
    private static int pipeLine22 = 8;
    private static int pipeLine23 = 9;

    private VoltageSensor VoltSens;

    //BlinkinLEDs
    private RevBlinkinLedDriver blinkinLedDriver;

    /***************************/
    /* Possible Automous Modes */
    /***************************/
    public enum AutoMode
    {
        AUTO_MODE_NOT_SELECTED,
        AUTO_MODE_BLUE_GOAL,
        AUTO_MODE_BLUE_AUDIENCE,
        AUTO_MODE_RED_GOAL,
        AUTO_MODE_RED_AUDIENCE
    }

    /***************************/
    /* Possible Park Location  */
    /***************************/
    public enum motifs
    {
        MOTIF_PATTERN_NOT_SELECTED,
        MOTIF_PATTERN_PPG,           // Apriltag #23
        MOTIF_PATTERN_PGP,           // Apriltag #22
        MOTIF_PATTERN_GPP            // Apriltag #21
    }

    /**********************/
    /* Sorter Directions  */
    /**********************/
    public enum sortDir
    {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }

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
    private RevBlinkinLedDriver.BlinkinPattern BasePattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    private RevBlinkinLedDriver.BlinkinPattern PPGmotif = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private RevBlinkinLedDriver.BlinkinPattern PGPmotif = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    private RevBlinkinLedDriver.BlinkinPattern GPPmotif = RevBlinkinLedDriver.BlinkinPattern.GREEN;


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
    AutoMode   autoMode = AutoMode.AUTO_MODE_NOT_SELECTED;         // store automous mode selected
    motifs     motifPattern = motifs.MOTIF_PATTERN_NOT_SELECTED;  // store park location selected

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

    private AutoMode SelectAutoMode()
    {
        AutoMode autoMode = AutoMode.AUTO_MODE_NOT_SELECTED;      // Local variable to store selected automous mode

        /*******************************************/
        /* Display automous mode not selected yet  */
        /*******************************************/
        telemetry.addData("AutoMode","Not Selected");
        telemetry.addData("AutoMode","Select a - Red Goal");
        telemetry.addData("AutoMode","Select b - Red Audiance");
        telemetry.addData("AutoMode","Select x - Blue Goal");
        telemetry.addData("AutoMode","Select y - Blue Audiance");

        telemetry.update();

        /*****************************************/
        /* Loop until automous mode is selected  */
        /*****************************************/
        while (!isStopRequested() && autoMode == AutoMode.AUTO_MODE_NOT_SELECTED)  {

            if (gamepad1.a) {  // Red side away from audience/Blue side toward audience
                autoMode = AutoMode.AUTO_MODE_RED_GOAL;
            }

            if (gamepad1.b) {  // Red side toward audience/Blue side away from audience
                autoMode = AutoMode.AUTO_MODE_RED_AUDIENCE;
            }

            if (gamepad1.x) {  // Red side away from audience/Blue side toward audience
                autoMode = AutoMode.AUTO_MODE_BLUE_GOAL;
            }

            if (gamepad1.y) {  // Red side toward audience/Blue side away from audience
                autoMode = AutoMode.AUTO_MODE_BLUE_AUDIENCE;
            }

            idle();
        }

        /************************************/
        /* Display selected automous mode   */
        /************************************/
        // mode.setValue(autoMode.toString());
        telemetry.addData("Automous Mode", autoMode.toString());
        telemetry.update();

        // Wait for the user to release the button
        while (!isStopRequested() && (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y)) {
            idle();
        }

        return autoMode;

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
        double tgtPower = 0;

        //****************************************//
        // Map all robot hardware                 //
        //****************************************//

        pinpoint = new PinpointOdometry(hardwareMap);
        pinpoint.setPose(0.0, 0.0, 0.0);

        telemetry.addLine("Pinpoint Odometry Initialized!");
        telemetry.addLine("Keep robot still until start.");
        telemetry.update();

        // Mapped drive train wheels
        frontLeftDrive = hardwareMap.get(DcMotor.class,"FLMoto");
        frontRightDrive = hardwareMap.get(DcMotor.class,"FRMoto"); //Was BR
        backLeftDrive = hardwareMap.get(DcMotor.class,"BLMoto");
        backRightDrive = hardwareMap.get(DcMotor.class,"BRMoto"); //Was FR

        // map motors for ball intake and ball sorter
        intakeMoto = hardwareMap.get(DcMotor.class,"intake");


        //map sensor
        color = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        color.setGain(10);  // increase sensitivity of the color sensor

        VoltSens = hardwareMap.voltageSensor.get("Control Hub");

        // Blinkin - map LED lights
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
        blinkinLedDriver.setPattern(BasePattern);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */

        telemetry.addData("Mode","calibrating imu...." );
        telemetry.update();


        // Map LimeLight
        limeLight = hardwareMap.get(Limelight3A.class, "limelight");


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


        // Create local variable to store selected automous mode
        // and amount of delay time
        Integer delayTimeMilliseconds = 0;      // variable to store how long to delay before starting automous

        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        /*********************************************/
        /* Select Automous Mode, and Delay time      */
        /*********************************************/
        autoMode = SelectAutoMode();
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
        limeLight.start();

        resetEncoders();

        // Delay start if needed
        if (delayTimeMilliseconds > 0) {
            telemetry.addData("Status","Delaying");    // display delay status
            telemetry.update();

            sleep(delayTimeMilliseconds);             // wait selected amount of time

            telemetry.addData("Status","Running");  // dispay delay over and automous code is running
            telemetry.update();
        }

        telemetry.clearAll();               // clear display messages

        switch (autoMode) {
            case AUTO_MODE_RED_GOAL:
                RedGoal();
                break;

            case AUTO_MODE_BLUE_GOAL:
                BlueGoal();
                break;

            case AUTO_MODE_RED_AUDIENCE:
                RedAudience();
                break;

            case AUTO_MODE_BLUE_AUDIENCE:
                BlueAudience();
                break;

            case AUTO_MODE_NOT_SELECTED:
                // This one should not happen if it does do nothing
                break;
        }


    }

    /***************************************************************************/
    /* Function: Red Goal                                                      */
    /* Returns: none                                                           */
    /* Uses Global Variables: motifPattern                                     */
    /*                                                                         */
    /* This function is for when the robot is on the red alliance and starts   */
    /* in the goal location                                                    */
    /*                                                                         */
    /* The robot will be preloaded with a purple ball in the shooting positon  */
    /* and the green ball to the left and the other purple ball to the right.  */
    /* The limeLight will read the motif pattern which will be stored in a     */
    /* global variable.  Based on the pattern, the balls will be sorted,       */
    /* shot and the robot moved out of the shooting zone                       */
    /***************************************************************************/
    private void RedGoal() {

        // strafe to the left to read obsilk
        strafeLeft(60.0, halfPower);
        sleep(500);

        driveBackward(25.0, halfPower);
        sleep(250);

        // rotate to face goal
        rotate(16, halfPower);

        sleep(500);

        // determine pattern of the ball
        motifPattern = getMotif();

        // rotate to face goal
        rotate(-75, halfPower);


        // Shoot the balls based off of the motife read at the start of auto
        shootAllBalls(motorSpeedBlueGoal);

        // Drive to move off of line
        driveForward(5.0, halfPower);

    }  // End of RedGoal()


    /***************************************************************************/
    /* Function: Blue Goal                                                      */
    /* Returns: none                                                           */
    /* Uses Global Variables: motifPattern                                     */
    /*                                                                         */
    /* This function is for when the robot is on the blue alliance and starts   */
    /* in the goal location                                                    */
    /*                                                                         */
    /* The robot will be preloaded with a purple ball in the shooting positon  */
    /* and the green ball to the left and the other purple ball to the right.  */
    /* The limeLight will read the motif pattern which will be stored in a     */
    /* global variable.  Based on the pattern, the balls will be sorted,       */
    /* shot and the robot moved out of the shooting zone                       */
    /***************************************************************************/
    private void BlueGoal() {

        // strafe to the right to read obsilk
        strafeRight(60.0, halfPower);
        sleep(500);

        driveBackward(25.0, halfPower);
        sleep(250);

        // rotate to face goal
        rotate(-16, halfPower);

        sleep(500);

        // determine pattern of the ball
        motifPattern = getMotif();

        // rotate to face goal
        rotate(70, halfPower);


        // Shoot the balls based off of the motife read at the start of auto
        shootAllBalls(motorSpeedBlueGoal);

        // Drive to move off of line
        driveForward(5.0, halfPower);

    }  // End of BlueGoal()


    /***************************************************************************/
    /* Function: Blue Audience                                                  */
    /* Returns: none                                                           */
    /* Uses Global Variables: motifPattern                                     */
    /*                                                                         */
    /* This function is for when the robot is on the red alliance and starts   */
    /* in the shooting zone by the audiance                                    */
    /*                                                                         */
    /* The robot will be preloaded with a purple ball in the shooting positon  */
    /* and the green ball to the left and the other purple ball to the right.  */
    /* The limeLight will read the motif pattern which will be stored in a     */
    /* global variable.  Based on the pattern, the balls will be sorted,       */
    /* shot and the robot moved out of the shooting zone                       */
    /***************************************************************************/
    private void BlueAudience() {
        //time for limelight to start
        sleep(1000);

        // determine pattern of the ball
        motifPattern = getMotif();

        driveForward(12.0, 0.15);
        sleep(100);

        // rotate to face goal
        rotate(14, halfPower);

        // Shoot the balls based off of the motife read at the start of auto
        shootAllBalls(motorSpeedBlueAud);

        // Drive to move off of line
        driveForward(18.0, 0.2);

    } // End of BlueAudiance()

    /***************************************************************************/
    /* Function: Red Audiance                                                  */
    /* Returns: none                                                           */
    /* Uses Global Variables: motifPattern                                     */
    /*                                                                         */
    /* This function is for when the robot is on the red alliance and starts   */
    /* in the shooting zone by the audiance                                    */
    /*                                                                         */
    /* The robot will be preloaded with a purple ball in the shooting positon  */
    /* and the green ball to the left and the other purple ball to the right.  */
    /* The limeLight will read the motif pattern which will be stored in a     */
    /* global variable.  Based on the pattern, the balls will be sorted,       */
    /* shot and the robot moved out of the shooting zone                       */
    /***************************************************************************/
    private void RedAudience() {
        //time for limelight to start
        sleep(1000);

        // determine pattern of the ball
        motifPattern = getMotif();

        driveForward(12.0, 0.15);
        sleep(100);

        // rotate to face goal
        rotate(-15, halfPower);

        // Shoot the balls based off of the motife read at the start of auto
        shootAllBalls(motorSpeedRedAud);

        // Drive to move off of line
        driveForward(15.0, halfPower);

    } // End of RedAudiance()



    /************************************************************/
    /* Function: getMotif                                       */
    /* Returns: motif Pattern                                   */
    /*                                                          */
    /*          MOTIF_PATTERN_PPG,       // Apriltag #23        */
    /*          MOTIF_PATTERN_PGP,       // Apriltag #22        */
    /*          MOTIF_PATTERN_GPP        // Apriltag #21        */
    /*                                                          */
    /* This function is called to determine the motif patter    */
    /* for shooting the balls.                                  */
    /*                                                          */
    /************************************************************/
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

    /************************************************************/
    /* Function: checkTarget                                    */
    /* Returns: motif Pattern                                   */
    /*                                                          */
    /*          MOTIF_PATTERN_PPG,       // Apriltag #23        */
    /*          MOTIF_PATTERN_PGP,       // Apriltag #22        */
    /*          MOTIF_PATTERN_GPP        // Apriltag #21        */
    /*                                                          */
    /* This function is called to determine if the target is    */
    /* detected and will return true or false                   */
    /*                                                          */
    /************************************************************/
    private boolean checkTarget() {

        LLResult latestResult = limeLight.getLatestResult();

        // telemetry.addData("Pointer Null", latestResult);
        // telemetry.addData("Results Valid", latestResult.isValid());
        // telemetry.update();

        if (latestResult != null && latestResult.isValid()) {
            return true;
        }
        else
        {
            return false;

        }
    }

    /************************************************************/
    /* Function: shootAllBalls                                  */
    /* Global: motifPattern                                     */
    /* Returns: none                                            */
    /*                                                          */
    /*          MOTIF_PATTERN_NOT_SELECTED  - Default to 23     */
    /*          MOTIF_PATTERN_PPG,          - Apriltag #23      */
    /*          MOTIF_PATTERN_PGP,          - Apriltag #22      */
    /*          MOTIF_PATTERN_GPP           - Apriltag #21      */
    /*                                                          */
    /* This function is called to shoot the balls based off of  */
    /* the motif pattern.                                       */
    /*                                                          */
    /* Note: the preloaded balls should be purple at the back   */
    /* purple to the right and green to the left                */
    /*                                                          */
    /************************************************************/

    void shootAllBalls(double flyWheelPower) {

        // start flywheels


        sleep(250);

        switch (motifPattern) {

            case  MOTIF_PATTERN_NOT_SELECTED:
            case  MOTIF_PATTERN_PPG:

                // shoot first purple ball
                shootOneBall();

                // rotate to shoot second purple ball
                rotateSorter(sortDir.CLOCKWISE, 660);
                sleep(250);

                // shoot second purple ball
                shootOneBall();

                // rotate to shoot green ball
                rotateSorter(sortDir.CLOCKWISE, 650);
                sleep(300);

                // shoot green ball
                shootOneBall();

                // turn off flywheels


                break;

            case  MOTIF_PATTERN_PGP:

                // shoot first purple ball
                shootOneBall();

                // rotate to shoot green ball
                rotateSorter(sortDir.COUNTER_CLOCKWISE,800);
                sleep(260);

                // shoot green ball
                shootOneBall();

                // rotate to shoot second purple ball
                rotateSorter(sortDir.COUNTER_CLOCKWISE,645);
                sleep(260);

                // shoot second purple ball
                shootOneBall();

                // turn off fly wheels


                break;

            case  MOTIF_PATTERN_GPP:

                // rotate to shoot second purple ball
                rotateSorter(sortDir.COUNTER_CLOCKWISE, 690);
                sleep(250);

                // shoot green ball
                shootOneBall();

                // rotate to shoot first ball
                rotateSorter(sortDir.COUNTER_CLOCKWISE, 670);
                sleep(250);

                // shoot first purple ball
                shootOneBall();

                // rotate to shoot second purple ball
                rotateSorter(sortDir.COUNTER_CLOCKWISE, 640);
                sleep(250);

                // shoot second purple ball
                shootOneBall();

                // turn off fly wheels


                break;

        }

    }  // end shootAllBalls

    /************************************************************/
    /* Function: shootOneBalls                                  */
    /* Global: none                                             */
    /* Returns: none                                            */
    /*                                                          */
    /*          MOTIF_PATTERN_NOT_SELECTED  - Default to 23     */
    /*          MOTIF_PATTERN_PPG,          - Apriltag #23      */
    /*          MOTIF_PATTERN_PGP,          - Apriltag #22      */
    /*          MOTIF_PATTERN_GPP           - Apriltag #21      */
    /*                                                          */
    /* This function is called to shoot one ball                */
    /*                                                          */
    /*                                                          */
    /************************************************************/

    void shootOneBall() {

        //lifter goes up to push ball up


        sleep(500);

        //flipper pushes to launcher


        sleep(250);

        //flipper resets to init
        sleep (200);

        //lifter comes down to init

        sleep (200);

    } // end shootOneBall

    /************************************************************/
    /* Function: rotateSorter                                   */
    /* Global: none                                             */
    /* Returns: none                                            */
    /*                                                          */
    /* This function is called to rotate the sorter a given     */
    /* amount of time                                           */
    /*                                                          */
    /************************************************************/
    void rotateSorter(sortDir direction, int delay) {

        //sorterMoto.setTargetPosition(postition);
        //sorterMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (direction == sortDir.CLOCKWISE) {

        }
        else {

        }

        sleep(delay);

        //while(opModeIsActive() && sorterMoto.isBusy()) {
        //    ;
        //}

        // stop sorter motor


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

    private void setDrivePower(double FLpower,double FRpower, Double BLpower, Double BRpower)
    {
        frontRightDrive.setPower(FRpower);
        backRightDrive.setPower(BRpower);
        frontLeftDrive.setPower(FLpower);
        backLeftDrive.setPower(BLpower);
    }

    private double checkDirection () {
        pinpoint.update();

        double currentHeading = pinpoint.getHeadingDegrees();
        double error = normalizeHeading(-currentHeading);

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
    private void strafeRight(Double inches,Double power)
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
    private void strafeLeft(Double inches,Double power)
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
    private void driveForward(Double inches,Double power)
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
    private void driveBackward(Double inches,Double power)
    {
        driveInches(inches, -power, -power, -power, -power);
    }

    private double normalizeHeading(double angle) {
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
    private void driveInches(Double inches,Double FLpower,Double FRpower, Double BLpower, Double BRpower) {
        {

            double counts = Math.abs(inches * countsPerInch);

            resetEncoders();
            runUsingEncoders();

            voltage = VoltSens.getVoltage();

            // Prevent division by zero or invalid voltage readings.
            if (voltage <= 0) {
                setDrivePower(0.0, 0.0, 0.0, 0.0);
                return;
            }

            FLpower = (batteryConst * FLpower) / voltage;
            FRpower = (batteryConst * FRpower) / voltage;
            BLpower = (batteryConst * BLpower) / voltage;
            BRpower = (batteryConst * BRpower) / voltage;

            setDrivePower(FLpower, FRpower, BLpower, BRpower);

            ElapsedTime driveTimer = new ElapsedTime();

            // Safety timeout: stop if the requested movement takes too long.
            double timeoutSeconds = 5.0 + Math.abs(inches) / 5.0;

            while (opModeIsActive()
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

                correction = checkDirection();

                setDrivePower(
                        FLpower - correction,
                        FRpower + correction,
                        BLpower - correction,
                        BRpower + correction
                );

                idle();
            }

            // Always stop the drive motors when the loop ends.
            setDrivePower(0.0, 0.0, 0.0, 0.0);

            runUsingEncoders();    // Stop all motors
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
    }
    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpoint.update();

        double targetHeading = normalizeHeading(pinpoint.getHeadingDegrees() + degrees
        );

        ElapsedTime rotateTimer = new ElapsedTime();
        double timeoutSeconds = 5.0;

        while (opModeIsActive()
                && rotateTimer.seconds() < timeoutSeconds) {

            pinpoint.update();

            double currentHeading = normalizeHeading(
                    pinpoint.getHeadingDegrees()
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

            telemetry.addData("Target Heading", "%.1f", targetHeading);
            telemetry.addData("Current Heading", "%.1f", currentHeading);
            telemetry.addData("Heading Error", "%.1f", error);
            telemetry.update();

            idle();
        }

        setDrivePower(0.0, 0.0, 0.0, 0.0);

        runUsingEncoders();

        sleep(250);
    }



}

