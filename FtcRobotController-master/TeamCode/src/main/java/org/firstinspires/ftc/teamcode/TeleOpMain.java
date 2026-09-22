package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.regex.Pattern;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.graphics.Color;

// Blinkin
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


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


// LimeLight
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.List;

//banana peanut butter
@TeleOp(name = "TeleOpMain (Blocks to Java)", group = "Drive")
public class TeleOpMain extends LinearOpMode {

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
  private Servo pushBallBack;
  private Servo lift;
  private Servo flipper;
  
  // This declares the IMU needed to get the current direction the robot is facing
  IMU imu;
    
  // Create color sensor  
  private NormalizedColorSensor color;
  
  // Create Limelight
  private Limelight3A limeLight;
  private int pipeLineGoalTags = 0;
  
  // Declare static variables
  private double reduceSpeed = 1.0;
  
  private static double powerConstant = .51;
  
  // set lift servo down and up positions
  // Larger number is down and small numbers raise lift higher
  private static double liftDownPosition = 0.9;
  private static double liftUpPosition = 0.53;
  
  //set pushBallFront servo down and up positions
  // note: closer to 1, the lower the height
  private static double pushBallFUpPos = 0.5;
  private static double pushBallFSortPos = 0.6;
  private static double pushBallFDownPos = 1.0;
  
  //set pushBallBack servo down and up positions
  private static double pushBallBUpPos = 0.3;
  private static double pushBallBDownPos = .6;
  // set positions for flipper to shoot ball
  private static double flipPositionInit = 0.0;
  private static double flipPositionShoot = 0.7;
  
  private static double sorterPowerSpeed = 0.4;
  private static double sorterPowerSpeedIntake = 0.35;
  private static double sorterPowerSpeedReset = 1.0;
  private static int rotateSorterRotateDelay = 200;
  private static double sorterPowerSpeedStop = 0.0; 
 
  private static int sorterIntakePos = 0;

  // check for color ball - norm colors for green and red RBG

  //***********************************************************
  /* Calibration Numbers for color sensor values in classroom

         Note: need to be calibrated at each event!!!
      
         Green Ball:
              Red   Min: 0.160      Max: 0.223
              Green Min: 0.200    Max: 0.297
              Blue  Min: 0.240    Max: 0.297
 
         Purple Ball: 
              Red   Min: 0.294      Max: 0.308
              Green Min: 0.318      Max: 0.333
              Blue  Min: 0.170     Max: 0.172
 
         No Ball: 
              Red   Min: 0.122     Max: 0.125
              Green Min: 0.144     Max: 0.146
              Blue  Min: 0.077    Max: 0.077
             
  */
  //***********************************************************

  private static double normGreenforGreenBall = 0.2; // greater than
  private static double normRedforGreenBall = 0.5; // greater than

  private static double normGreenforPurpleBall = 0.45; // greater than
  private static double normRedforPurpleBall = 0.5;  // less than
  
  private double targetVelocity = 1620; // target velocity in RPM
  
  // Blinkin
  private RevBlinkinLedDriver blinkinLedDriver;
  
  private RevBlinkinLedDriver.BlinkinPattern BasePattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
  private RevBlinkinLedDriver.BlinkinPattern PurpleBall = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
  private RevBlinkinLedDriver.BlinkinPattern GreenBall = RevBlinkinLedDriver.BlinkinPattern.GREEN;

  private double wheelPower = 0.0;
  private static double scaleForDist = 171.5592;
  
  protected enum DisplayKind {
        MANUAL,

        AUTO
  }

  void driveFieldRelative(double forward, double strafe, double rotate) {
      //First, convert diretion being asked to drive to polar cordinates
      double theta = Math.atan2(forward, strafe);
      double r = Math.hypot(strafe, forward);
      //Second, rotate angle by the angle the robot is pointing
      theta = AngleUnit.normalizeRadians(theta -
              imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
      //Third, convert back to cartesian
      double newForward = r * Math.sin(theta);
      double newStrafe = r * Math.cos(theta);
      //Fianlly, call the drive method with robot relative forward and right amounts
      drive(newForward, newStrafe, rotate);
  }

  void drive(double forward, double strafe, double rotate) {
      //This calculate the power needed for each wheel based on the amount forward ,
      double frontLeftPower = forward + strafe + rotate;
      double frontRightPower = forward - strafe - rotate;
      double backRightPower = forward + strafe - rotate;
      double backLeftPower = forward - strafe + rotate;

      double maxPower = 1.0;
      double maxSpeed = 1.0;

      maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
      maxPower = Math.max(maxPower, Math.abs(frontRightPower));
      maxPower = Math.max(maxPower, Math.abs(backRightPower));
      maxPower = Math.max(maxPower, Math.abs(backLeftPower));

      frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
      frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
      backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
      backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    
    // Variables to toggle the on/off of the fly wheel motors
    boolean motorOn = false;
    boolean lastMotorState = false;
    int currentSorterPos = 0;
    
    

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
    
    color = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    color.setGain(3);  // increase sensitivity of the color sensor
    
    // velocityController = new PIDFController (
    //    new P(0.1), // P gain
    //    new I(0.0), // I gain
    //    new D(0.0), // D gain
    //    new F(0.0)  // F gain 
    //    );
    
    // set the target velocity for the flywheel
    // volocityController.setTargetVelocity(targetVelocity);

      // map ALL servos
    lift = hardwareMap.servo.get("lift");
    pushBallFront = hardwareMap.servo.get("pushBall");
    pushBallBack = hardwareMap
            .servo.get("pushballback");
    flipper = hardwareMap.servo.get("flipper");
    
    // Blinkin - map LED lights
    blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
    blinkinLedDriver.setPattern(BasePattern);

    // map and intiialize IMU on control hub for field relative driving.
        imu = hardwareMap.get(IMU.class, "imu");
        
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

      RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
              RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
              RevHubOrientationOnRobot.UsbFacingDirection.UP);

      imu.initialize(new IMU.Parameters(RevOrientation));
        
    // Map LimeLight
    limeLight = hardwareMap.get(Limelight3A.class, "limelight");
    limeLight.pipelineSwitch(pipeLineGoalTags); // need to setup pipeline for april tags
    
   

    //*****************************************//
    // Put initialization blocks here.         //
    //*****************************************//
        
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
    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    intakeMoto.setDirection(DcMotorSimple.Direction.REVERSE);
    intakeMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    intakeMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    sorterMoto.setDirection(DcMotorSimple.Direction.FORWARD);     
    //sorterMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    sorterMoto.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //sorterMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //sorterMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //sorterMoto.setTargetPosition(0);

    flyWheelRightMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    flyWheelRightMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    flyWheelRightMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    flyWheelLeftMoto.setDirection(DcMotorSimple.Direction.REVERSE);
    flyWheelLeftMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    flyWheelLeftMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Initialize the start positions and direction for servos
    //lift.setDirection(Servo.Direction.REVERSE);
    lift.setPosition(liftDownPosition);
    pushBallFront.setPosition(pushBallFDownPos);
    pushBallBack.setPosition(pushBallBDownPos);
    flipper.setPosition(flipPositionInit);
    

    
    /*
     * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
     * The limeLight will drain the battery so since this is teleop, we can start the limeLight and 
     * don't need to wait for the start button to be pressed.
    */
    limeLight.start();

    telemetry.addData(">", "Robot Ready.  Press Play.");
    telemetry.update();

    telemetry.addData("Pattern: ", BasePattern.toString());
    telemetry.update();
    
    // Wait for the start of TeleOp
    waitForStart();
    
    // Put run blocks here.

    //******************************************//
    // Run code while op mode is active         //
    //******************************************//
    while (opModeIsActive()) 
    {

      
        NormalizedRGBA ballColor = color.getNormalizedColors();
        
        float normRed, normGreen, normBlue;

        normRed = ballColor.red /ballColor.alpha;
        normGreen = ballColor.green /ballColor.alpha;
        normBlue = ballColor.blue /ballColor.alpha;
        
        telemetry.addLine();
        telemetry.addData("Red", "%.3f", normRed);
        telemetry.addData("Green", "%.3f", normGreen);
        telemetry.addData("Blue", "%.3f", normBlue);
        telemetry.update();

        // Determine if ball present and if so what color
        if (normGreen > normGreenforGreenBall && normRed > normRedforGreenBall) {
            blinkinLedDriver.setPattern(GreenBall);
        }
        else if (normGreen > normGreenforPurpleBall && normRed < normRedforPurpleBall) {
          blinkinLedDriver.setPattern(PurpleBall);
        } 
        else {
            blinkinLedDriver.setPattern(BasePattern);
        }

       //Lift ball for shooting
       if (gamepad1.a)
       {

          //lifter goes up
          lift.setPosition(liftUpPosition);
       
          sleep(500);
          pushBallBack.setPosition(pushBallBDownPos);
          //flipper pushes to launcher
          flipper.setPosition(flipPositionShoot);
          
          sleep(250);
          
          //flipper resets to init
          flipper.setPosition(flipPositionInit);

          //lifter comes down to init
          lift.setPosition(liftDownPosition);

       } 
       
      // push ball front flap up during intake
      if(gamepad1.dpad_up)
      {
          pushBallFront.setPosition(pushBallFSortPos);
          sleep(100);
          
          sorterMoto.setPower(sorterPowerSpeedIntake);          
          sleep(rotateSorterRotateDelay);
          sorterMoto.setPower(sorterPowerSpeedStop);
          
      }
      
      // push ball front flap up sorting setting for rotating the balls
      if(gamepad1.dpad_right)
      {
          pushBallFront.setPosition(pushBallFSortPos);
      }
      
      // reset pushball up front flap down to init state
      if(gamepad1.dpad_down)
      {
          pushBallFront.setPosition(pushBallFDownPos);
      }
      
        //intake spins inwards
       if(gamepad1.left_bumper)
       {
          intakeMoto.setPower(1.0);
          pushBallFront.setPosition(pushBallFDownPos);
       } 
       else if (gamepad1.right_bumper)
       {
          intakeMoto.setPower(-1.0);
          pushBallFront.setPosition(pushBallFDownPos);
       }
       else
       {
          intakeMoto.setPower(0.0);

       } 
       
      
      //
      // code to toogle the on and off of the fly wheels using the "b" button on game pad 1
      // 

      // read current state of button "b"
      boolean currentMotorState = gamepad1.b;
      
      // check if the button was just pressed
      // compare current state of the button to the last state of the button
      // which helps determine if the motor needs to be turned on or off
      if (currentMotorState && !lastMotorState) {
          // Toggle the motor state
          motorOn = !motorOn;
      }
      
      // Always update the last button/motor state for the next loop
      lastMotorState = currentMotorState;
      
      // control the motor based on the toggle state
      if(motorOn){
             
          LLResult lastResult = limeLight.getLatestResult();
          
          if (lastResult != null && lastResult.isValid()) {
                          
             double targetArea = lastResult.getTa();
       
             telemetry.clearAll();
             
             telemetry.addData("Target Area",targetArea );
             telemetry.update();
             
             if (targetArea > 3.5){
               wheelPower = 0.90;
             }
             else if ((targetArea > 2.0) && (targetArea < 3.5  )){
               wheelPower = 0.7;
             }
             else if (targetArea < 2.0 && targetArea > 0.90){
               wheelPower = 0.75;
             }
             else {
               wheelPower = 0.90;
             }
             
             telemetry.clearAll();
             
             telemetry.addData("Target Area", targetArea );
             telemetry.addData("Wheel Speed", wheelPower );
             telemetry.update();
          }
          else{
            wheelPower = .85;
          }

          // turn on fly wheels           telemetry.clearAll();
          flyWheelRightMoto.setPower(wheelPower);
          flyWheelLeftMoto.setPower(wheelPower); 
      }
      else {
          flyWheelRightMoto.setPower(0.0);
          flyWheelLeftMoto.setPower(0.0);
      }
       
      // Sort ball clockwise 
      if(gamepad1.y){
         sorterMoto.setPower(sorterPowerSpeed);
         
          // delay request amout of time;
          sleep(rotateSorterRotateDelay);

          // stop sorter motor
          sorterMoto.setPower(sorterPowerSpeedStop);

      }

      // Sort ball counter clockwise 
      if(gamepad1.x){
         sorterMoto.setPower(-sorterPowerSpeed);
         
         //delay request amout of time;
          sleep(rotateSorterRotateDelay);

         // stop sorter motor
          sorterMoto.setPower(sorterPowerSpeedStop); 
          
          
          
          sorterMoto.setPower(0);
      }

    //Movement

        driveFieldRelative(gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);



        
    }
    
  }
}

//***********************************************************
// GamePad 1 Usage
//
//  Axis
//   left_stick_x = not used
//   left_stick_y = rotate
//   right_stick_x = strafe drive
//   right_stick_y = forward drive
//
//  Buttons
//    a = lift ball and shootzzz
//    x = sorter moves to 50 (first ball) 
//    y = sorter moves to 140 (second ball)
//    left_bumper = intake motor on/off
//    right_bumper = strafe right
//    left_trigger = strafe left
//    right_trigger = intake motor spin outtake
//    back = sorter moves back to zero
//    start = sorter moves thrid part
//  
//  hat
//    dpad_up = move flipper for intake up
//    dpad_down = move flipper for intake all the way down
//    dpad_right = move flipper for intake part way for sorting
//    dpad_left = not used
  
