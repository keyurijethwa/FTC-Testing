package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Touch Sensor Test")
public class TouchSensorTest extends LinearOpMode {

    private TouchSensor touchSensor;

    @Override
    public void runOpMode() {

        // Make sure the sensor is named "touch" in Robot Configuration
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        waitForStart();

        while (opModeIsActive()) {

            if (touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "PRESSED");
            } else {
                telemetry.addData("Touch Sensor", "RELEASED");
            }

            telemetry.update();
        }
    }
}