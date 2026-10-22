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

public class AutoMain extends LinearOpMode {

    // Drive Wheels
    // Create Drive Wheel motors
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // Create motors and velocity controller for flywheels
    private DcMotorEx flyWheelRightMoto;
    private DcMotorEx flyWheelLeftMoto;
    // private PIDFController velocityController;

    // Create other motors
    //
    private DcMotor intakeMoto;
    private DcMotor sorterMoto;

    // Create all servos
    private Servo pushBallFront;
    private Servo lift;
    private Servo flipper;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
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
    private static final Double straifFactor = 1.0;
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
    private static double liftDownPosition = 0.9;
    private static double liftUpPosition = 0.53;

    //set pushBallFront servo down and up positions
    // note: closer to 1, the lower the flipper
    private static double pushBallFUpPos = 0.5;
    private static double pushBallFSortPos = 0.61;
    private static double pushBallFDownPos = 1.0;

    // set positions for flipper to shoot ball
    private static double flipPositionInit = 0.0;
    private static double flipPositionShoot = 0.7;

    private static double sorterPowerSpeed = 0.30;
    private static int rotateSorterRotateDelay = 215;
    private static int sorterPowerSpeedStop = 0;

    private static int sorterIntakePos = 0;

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
    //***********************************************************

    private static double normGreenforGreenBall = 0.7;  // greater than
    private static double normRedforGreenBall = 0.5;   // less than

    private static double normGreenforPurpleBall = 0.3; // greater than
    private static double normRedforPurpleBall = 0.5;  // greater than


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

        // Mapped drive train wheels
        frontLeftDrive = hardwareMap.get(DcMotor.class,"FLMoto");
        frontRightDrive = hardwareMap.get(DcMotor.class,"FRMoto"); //Was BR
        backLeftDrive = hardwareMap.get(DcMotor.class,"BLMoto");
        backRightDrive = hardwareMap.get(DcMotor.class,"BRMoto"); //Was FR

        // map motors for ball intake and ball sorter
        intakeMoto = hardwareMap.get(DcMotor.class,"intake");
        sorterMoto = hardwareMap.get(DcMotor.class,"Sorter");


        // map fly wheels motors
        flyWheelRightMoto = hardwareMap.get(DcMotorEx.class,"flyWheelRight");
        flyWheelLeftMoto = hardwareMap.get(DcMotorEx.class,"flyWheelLeft");

        //********************/
        //* Map All Servos   */
        //********************/
        lift = hardwareMap.servo.get("lift");
        pushBallFront = hardwareMap.servo.get("pushBall");
        flipper = hardwareMap.servo.get("flipper");

        //map sensor
        color = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        color.setGain(10);  // increase sensitivity of the color sensor

        VoltSens = hardwareMap.voltageSensor.get("Control Hub");

        // Blinkin - map LED lights
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
        blinkinLedDriver.setPattern(BasePattern);

        /************************/
        /* Setup IMU parameters */
        /************************/

        // Retrieve and initialize the IMU. The IMU should be attached to
        // IC2 port 0 on a Core Device Interface Module
        imu = hardwareMap.get(IMU.class, "imu");

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        telemetry.addData("Mode","calibrating imu...." );
        telemetry.update();

        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));

            telemetry.addData("imu calib status", "calibrated" );
            telemetry.update();

            // initialize imu global variables after calibrating imu
            resetAngle();

        } catch (IllegalArgumentException e) {
            telemetry.addData("imu calib status", "failed - try again");
            telemetry.update();
        }

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

        sorterMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        sorterMoto.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorterMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // sorterMoto.setTargetPosition(0);

        flyWheelRightMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelRightMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelRightMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelRightMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheelLeftMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelLeftMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelLeftMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelLeftMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the start positions and direction for servos
        lift.setPosition(liftDownPosition);
        pushBallFront.setPosition(pushBallFSortPos);
        flipper.setPosition(flipPositionInit);

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

        // Initialize the start positions and direction for servos
        lift.setPosition(liftDownPosition);
        pushBallFront.setPosition(pushBallFDownPos);
        flipper.setPosition(flipPositionInit);

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
    /* Function: Blue Audiance                                                  */
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
        flyWheelRightMoto.setPower(flyWheelPower);
        flyWheelLeftMoto.setPower(flyWheelPower);

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
                flyWheelRightMoto.setPower(0.0);
                flyWheelLeftMoto.setPower(0.0);

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
                flyWheelRightMoto.setPower(0.0);
                flyWheelLeftMoto.setPower(0.0);

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
                flyWheelRightMoto.setPower(0.0);
                flyWheelLeftMoto.setPower(0.0);

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
        lift.setPosition(liftUpPosition);

        sleep(500);

        //flipper pushes to launcher
        flipper.setPosition(flipPositionShoot);

        sleep(250);

        //flipper resets to init
        flipper.setPosition(flipPositionInit);
        sleep (200);

        //lifter comes down to init
        lift.setPosition(liftDownPosition);
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
            sorterMoto.setPower(sorterPowerSpeed);
        }
        else {
            sorterMoto.setPower(-sorterPowerSpeed);
        }

        sleep(delay);

        //while(opModeIsActive() && sorterMoto.isBusy()) {
        //    ;
        //}

        // stop sorter motor
        sorterMoto.setPower(sorterPowerSpeedStop);

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


    /************************************************************/
    /* Function: strafeRight                                    */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move sideways  */
    /* in a right direction                                     */
    /************************************************************/
    private void strafeRight(Double inches,Double power)
    {
        driveInches(inches*straifFactor, power, -power, -power, power);

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
        driveInches(inches*straifFactor, -power, power, power, -power);
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
    private void driveInches(Double inches,Double FLpower,Double FRpower, Double BLpower, Double BRpower)
    {
        Double counts = inches * countsPerInch;

        resetEncoders();
        runUsingEncoders();

        voltage = VoltSens.getVoltage();  // read current battery voltage

        FLpower = ((batteryConst*FLpower)/voltage);
        FRpower = ((batteryConst*FRpower)/voltage);
        BLpower = ((batteryConst*BLpower)/voltage);
        BRpower = ((batteryConst*BRpower)/voltage);

        setDrivePower(FLpower, FRpower, BLpower, BRpower);

        while (opModeIsActive() &&
                (Math.abs(frontLeftDrive.getCurrentPosition()) + Math.abs(frontRightDrive.getCurrentPosition()) /2) < Math.abs(counts))
        {
            // Use gyro to drive in a straight line.
            // correction = checkDirection();

            correction = 0;

            //telemetry.addData("1 imu heading", lastAngles.firstAngle);
            // telemetry.addData("2 global heading", globalAngle);
            // telemetry.addData("3 correction", correction);
            // telemetry.update();

            //setDrivePower(FLpower+correction, FRpower-correction, BLpower+correction, BRpower-correction);
            setDrivePower(FLpower-correction, FRpower+correction, BLpower-correction, BRpower+correction);
            idle();
        }

        setDrivePower(noPower,noPower,noPower,noPower);       // Stop all motors
    }


    //***************************************************************************
    // Functions for turning and checking robot angle for correction
    //

    /***************************************************/
    /* Resets the cumulative angle tracking to zero.   */
    /***************************************************/
    private void resetAngle()
    {
        imu.resetYaw();
        lastAngles = imu.getRobotYawPitchRollAngles();

        globalAngle = 0;
    }

    /************************************************************/
    /* Get current cumulative angle rotation from last reset.   */
    /* @return Angle in degrees. + = left, - = right.           */
    /************************************************************/
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        // Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double deltaAngle = orientation.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = orientation;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.

        //Orientation angles = imu.getRobotYawPitchRollAngles();
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw();

        // Calculate the error
        double error = - currentHeading;

        // Normalize the error to be within -180 to 180 degrees
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        // Simple proportional control for correction
        // Adjust the 'Kp' (proportional constant) as needed for your robot
        double Kp = 0.02; // Example value, tune for your robot

        double correction = error * Kp;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double    leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        if (degrees < 0)
        {    // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {    // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        setDrivePower(leftPower,rightPower,leftPower,rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
        {
            while (opModeIsActive() && getAngle() < degrees) {}
        }

        // turn the motors off.
        setDrivePower(noPower,noPower,noPower,noPower);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

}
