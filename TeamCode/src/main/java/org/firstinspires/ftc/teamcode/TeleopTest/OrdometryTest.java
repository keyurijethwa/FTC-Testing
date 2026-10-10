
package org.firstinspires.ftc.teamcode.TeleopTest;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class OrdometryTest {

    GoBildaPinpointDriver pinpoint;

    public OrdometryTest(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint");

        configurePinpoint();

        pinpoint.setPosition(
                new Pose2D(DistanceUnit.INCH, 0, 0,
                        AngleUnit.DEGREES, 0));
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }

    public double getX() {
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }

    public double getY() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

}
