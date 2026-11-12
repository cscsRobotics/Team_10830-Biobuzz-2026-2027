package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ControlHubIMU {
    private final IMU imu;

    /**
     * Constructor to initialize and configure the Control Hub IMU.
     *
     * @param hwMap The hardwareMap passed from your OpMode
     * @param logoDirection The direction the REV logo is facing on the robot
     * @param usbDirection The direction the USB ports are facing on the robot
     */
    public ControlHubIMU(HardwareMap hwMap,
                         RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
                         RevHubOrientationOnRobot.UsbFacingDirection usbDirection) {

        // Retrieve the IMU from the hardware map (default config name is "imu")
        imu = hwMap.get(IMU.class, "imu");

        // Configure the orientation of the Control Hub relative to the robot chassis
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoDirection,
                usbDirection));

        imu.initialize(parameters);
    }

    /**
     * Resets the IMU's yaw (heading) to zero.
     */
    public void resetYaw() {
        imu.resetYaw();
    }

    /**
     * Gets the current robot heading in degrees (-180 to 180).
     */
    public double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Gets the current robot heading in radians (-PI to PI).
     */
    public double getHeadingRadians() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }
}