package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointOdometry {
    private final GoBildaPinpointDriver pinpoint;
    
    // Geometry Constants (Modify these to match your physical robot)
    private static final double X_OFFSET_MM = -185.0; // Distance from center of robot to X pod
    private static final double Y_OFFSET_MM = -133.0; // Distance from center of robot to Y pod
    
    /**
     * Constructor to initialize the Pinpoint Odometry Computer
     * @param hwMap The hardwareMap passed from your OpMode
     */
    public PinpointOdometry(HardwareMap hwMap) {
        // Retrieve device from configuration named "pinpoint"
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        
        // Configure Pinpoint hardware settings
        pinpoint.setOffsets(X_OFFSET_MM, Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        
        // Fully recalibrate gyro tracking and reset positioning axes
        // Note: Robot MUST be perfectly still during this instantiation step!
        pinpoint.resetPosAndIMU();
    }
    
    /**
     * Updates the internal positions. 
     * Call this exactly ONCE at the very top of your main execution loop.
     */
    public void update() {
        pinpoint.update();
    }
    
    /**
     * Replaces standard internal IMU heading retrieval.
     * @return Current robot heading in Radians (-PI to +PI)
     */
    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }
    
    /**
     * Replaces standard internal IMU heading retrieval.
     * @return Current robot heading in Degrees (-180 to +180)
     */
    public double getHeadingDegrees() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }
    
    /**
     * Returns the full Pose data bundle (X, Y, Heading)
     */
    public Pose2D getPose() {
        return pinpoint.getPosition();
    }
    
    /**
     * Gets X Position in Inches (useful for American FTC field tiles)
     */
    public double getXInches() {
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }
    
    /**
     * Gets Y Position in Inches 
     */
    public double getYInches() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }
    
    /**
     * Allows you to set a custom field position (e.g., at the start of Autonomous)
     */
    public void setPose(double xInches, double yInches, double headingRadians) {
        Pose2D startPose = new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.RADIANS, headingRadians);
        pinpoint.setPosition(startPose);
    }
}
