package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoClass {

    private Servo ballAdjustment; //port 1

    //set positions for ballAdjustment to move to
    private final double ballAdjustSmall = 0.25; //set values from testing
    private final double ballAdjustBig = 0.75; //set values from testing

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


