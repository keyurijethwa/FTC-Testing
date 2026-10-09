package org.firstinspires.ftc.teamcode.pedropath_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Intake_Balls;

@TeleOp(name = "Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {
    private static final double RGB_RED = 0.30;
    private static final double RGB_YELLOW = 0.35;
    private static final double RGB_GREEN = 0.50;
    private static final double RGB_BLUE = 0.55;

    Intake_Balls ib;
    @Override
    public void runOpMode() {

        DistanceSensor distanceSensor =
                hardwareMap.get(DistanceSensor.class, "DS");

        Servo s = hardwareMap.get(Servo.class,"rgb");
        ib=new Intake_Balls(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double distance =
                    distanceSensor.getDistance(DistanceUnit.CM);

            if (distance < 8) {
                s.setPosition(RGB_RED);
                ib.stop1();
                telemetry.addLine("RED - Object Close");
            } else if (distance >8) {
                s.setPosition(RGB_BLUE);
                ib.in(0.8);
                telemetry.addLine("BLUE - Object Medium Distance");
            } else {
                s.setPosition(RGB_GREEN);
                ib.stop1();
                telemetry.addLine("GREEN - Object Far");
            }

            telemetry.addData("Distance (cm)", distance);

            telemetry.update();

            sleep(100);
        }
    }
}
