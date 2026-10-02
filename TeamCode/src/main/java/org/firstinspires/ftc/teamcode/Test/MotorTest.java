package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Wire Test")
public class MotorTest extends LinearOpMode {

    private DcMotorEx motor;

    @Override
    public void runOpMode() {

        // Change "motor" to the name in your Robot Configuration
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Press a to give power:",0.5);
        waitForStart();

        while (opModeIsActive()) {

            // Hold A = motor forward
            if (gamepad1.a) {
                motor.setPower(0.5);
            } else {
                motor.setPower(0);
            }

            telemetry.addData("Motor velocity",motor.getVelocity());
            telemetry.addData("Power",motor.getPower());
            telemetry.update();
        }
    }
}