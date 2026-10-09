package org.firstinspires.ftc.teamcode.pedropath_test;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Intake_Balls;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "BioBuzzAutoPath10Blue", group = "Autonomous")
public class BioBuzzAutoPath10Blue extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(83, 133, 90);
    private final Pose path1 = poseFactory.of(83, 107, 90);
    private final Pose point2 = poseFactory.of(133.5708, 129.6702, 90);
    private final Pose point2Control1 = poseFactory.of(127.0552, 105.5407, 0);
    private final Pose point3 = poseFactory.of(86.922, 33.0159, 270);
    private final Pose point3Control1 = poseFactory.of(111.1374, 34.4164, 0);
    private final Pose point4 = poseFactory.of(132.1084, 93.3356, 0);
    private final Pose point4Control1 = poseFactory.of(104.8211, 32.107, 0);
    private final Pose point5 = poseFactory.of(86.5373, 33.4489, 270);
    private final Pose point5Control1 = poseFactory.of(103.2445, 29.9738, 0);
    private final Pose point6 = poseFactory.of(136.3198, 34.1816, 270);


    private ShootTest sb;
    private Intake_Balls ib;
    private Servo rgbLight;

    private static final double RGB_RED = 0.30;
    private static final double RGB_YELLOW = 0.35;
    private static final double RGB_GREEN = 0.50;
    private static final double RGB_BLUE = 0.55;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 1");
                }),

                follow(follower, path1()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - Shoot 1");
                }),

                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(3000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: Blue - PATH 2");
                }),

                follow(follower, path2()),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: Yellow - Intake 1");
                }),

                instant(()->ib.in(0.8)),
                waitMs(3000),
                instant(()->ib.stop1()),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: Blue - PATH 3");
                }),

                follow(follower, path3()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - Shoot 2");
                }),

                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(3000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: Blue - PATH 4");
                }),

                follow(follower, path4()),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: Yellow - Intake 2");
                }),

                instant(()->ib.in(0.8)),
                waitMs(3000),
                instant(()->ib.stop1()),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: Blue - PATH 5");
                }),

                follow(follower, path5()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - Shoot 3");
                }),

                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(3000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: Blue - PATH 6");
                }),

                follow(follower, path6())
        );
    }

    private void rgbRed() {
        rgbLight.setPosition(RGB_RED);
    }

    private void rgbYellow() {
        rgbLight.setPosition(RGB_YELLOW);
    }

    private void rgbGreen() {
        rgbLight.setPosition(RGB_GREEN);
    }

    private void rgbBlue() {
        rgbLight.setPosition(RGB_BLUE);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        ib=new Intake_Balls(hardwareMap);
        sb=new ShootTest(hardwareMap);
        rgbLight=hardwareMap.get(Servo.class,"rgb");

        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).linear(path1, point2);
    }

    public Path path3() {
        return curve(point2, point3Control1, point3).linear(point2, point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4).linear(point3, point4);
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).linear(point4, point5);
    }

    public Path path6() {
        return line(point5, point6).linear(point5, point6);
    }
}