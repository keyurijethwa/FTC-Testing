package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Wire Test")
public class ServoTest extends LinearOpMode {

    private Servo servo;

    @Override
    public void runOpMode() {

        // Make sure your servo is named "servo" in Robot Configuration
        servo = hardwareMap.get(Servo.class, "servo");
        telemetry.addData("Press a for set 0.0",0);
        telemetry.addData("Press b for set 1.0",1);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // Press A -> position 0
            if (gamepad1.a) {
                servo.setPosition(0.0);
            }

            // Press B -> position 1
            if (gamepad1.b) {
                servo.setPosition(1.0);
            }

            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}