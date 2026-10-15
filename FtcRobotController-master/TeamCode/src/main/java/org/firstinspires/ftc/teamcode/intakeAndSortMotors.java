package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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



// Blinkin


// Driving with Gyro (IMU) classes
//
// import for IMU (gyroscope)


public class intakeAndSortMotors {

    private DcMotor intakeMoto;
    private DcMotor sorterMoto;

    public void init(HardwareMap hardwareMap) {

        // map motors for ball intake and ball sorter
        intakeMoto = hardwareMap.get(DcMotor.class,"intake");
        sorterMoto = hardwareMap.get(DcMotor.class,"Sorter");


        intakeMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sorterMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        sorterMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMoto.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorterMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorterMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMoto.setTargetPosition(0);


    }


}
