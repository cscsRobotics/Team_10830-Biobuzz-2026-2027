package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoClass {

    private Servo ballAdjustment; //port 1

    //set positions for ballAdjustment to move to
    private static double ballAdjustSmall; //set values from testing
    private static double ballAdjustBig; //set values from testing

    private boolean smallBallPosition = true;

    public ServoClass(HardwareMap hardwareMap) {

        ballAdjustment = hardwareMap.get(Servo.class, "ballAdjustment");

        ballAdjustment.setPosition(ballAdjustSmall);
      }

    public void togglePosition() {

        if (smallBallPosition) {

            ballAdjustment.setPosition(ballAdjustBig);

        } else {

            ballAdjustment.setPosition(ballAdjustSmall);

        }

        smallBallPosition = !smallBallPosition;

    }

}


