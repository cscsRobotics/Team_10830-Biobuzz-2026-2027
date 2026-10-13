package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TestIMU;



public class imuPractice extends OpMode {

    TestIMU bench =  new TestIMU();

    @Override
    public void init(){
    bench.init(hardwareMap);

    }
    @Override
    public void loop(){
    telemetry.addData("heading", bench.getHeading(AngleUnit.RADIANS));

    }
}
