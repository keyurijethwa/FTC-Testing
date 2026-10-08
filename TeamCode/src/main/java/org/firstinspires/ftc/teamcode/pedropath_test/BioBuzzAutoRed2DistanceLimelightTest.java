package org.firstinspires.ftc.teamcode.pedropath_test;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Intake_Balls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedropath_test.ShootTest;

@Autonomous(name = "BioBuzzAutoRed2DistanceLimelightTest")
public class BioBuzzAutoRed2DistanceLimelightTest extends LinearOpMode {

    private Follower follower;
    private ShootTest sb;
    private Intake_Balls ib;
    private Servo rgbLight;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(60, 15, 90);

    private final Pose path1Start =
            poseFactory.of(60, 15, 270);

    private final Pose path1 =
            poseFactory.of(65, 41, 270);

    private final Pose path2 =
            poseFactory.of(15, 50, 180);

    private final Pose path3 =
            poseFactory.of(62, 41, 270);

    private final Pose path4Start =
            poseFactory.of(62, 41, 90);

    private final Pose path4 =
            poseFactory.of(62, 118, 90);

    private final Pose path5Start =
            poseFactory.of(62, 118, 90);

    private final Pose path5 =
            poseFactory.of(40, 138, 90);

    private final Pose path6Start =
            poseFactory.of(40, 138, 90);

    private final Pose path6 =
            poseFactory.of(62, 118, 90);

    private final Pose path7 =
            poseFactory.of(13, 110, 90);

    private static final double RGB_RED = 0.30;
    private static final double RGB_YELLOW = 0.35;
    private static final double RGB_GREEN = 0.50;
    private static final double RGB_BLUE = 0.55;

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

    public Command autoRoutine() {

        return sequential(

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - PATH 1");
                }),

                follow(follower, path1()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOT 1");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(2000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 2");
                }),

                follow(follower, path2()),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - INTAKE 1");
                }),

                instant(() -> ib.in(0.8)),

                waitMs(2000),

                instant(() -> ib.stop1()),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: BLUE - PATH 3");
                }),

                follow(follower, path3()),

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 4");
                }),

                follow(follower, path4()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOT 2");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(2000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - PATH 5");
                }),

                follow(follower, path5()),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - INTAKE 2");
                }),

                instant(() -> ib.in(0.8)),

                waitMs(2000),

                instant(() -> ib.stop1()),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: BLUE - PATH 6");
                }),

                follow(follower, path6()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOT 3");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(2000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 7");
                }),

                follow(follower, path7()),

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - AUTO COMPLETE");
                })
        );
    }

    @Override
    public void runOpMode() {

        ib = new Intake_Balls(hardwareMap);
        sb = new ShootTest(hardwareMap);

        rgbLight = hardwareMap.get(
                Servo.class,
                "rgb"
        );

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        follower.setPose(start);
        follower.update();

        rgbRed();

        telemetry.addLine("AUTO RED PATH 1");
        telemetry.addLine("RGB PATH INDICATOR");
        telemetry.addLine("RGB: RED");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        rgbRed();

        schedule(autoRoutine());

        while (opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            telemetry.addData(
                    "X",
                    "%.2f",
                    follower.pose().x()
            );

            telemetry.addData(
                    "Y",
                    "%.2f",
                    follower.pose().y()
            );

            telemetry.addData(
                    "Heading",
                    "%.2f",
                    follower.pose().heading()
            );

            if (follower.currentPath() != null) {
                telemetry.addData(
                        "Distance Remaining",
                        "%.2f",
                        follower.distanceToEndpoint()
                );

                telemetry.addData(
                        "Path Index",
                        follower.pathIndex()
                );
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(
                path1Start,
                path1
        ).linear(
                path1Start,
                path1
        );
    }

    public Path path2() {
        return line(
                path1,
                path2
        ).linear(
                path1,
                path2
        );
    }

    public Path path3() {
        return line(
                path2,
                path3
        ).linear(
                path2,
                path3
        );
    }

    public Path path4() {
        return line(
                path4Start,
                path4
        ).linear(
                path4Start,
                path4
        );
    }

    public Path path5() {
        return line(
                path5Start,
                path5
        ).linear(
                path5Start,
                path5
        );
    }

    public Path path6() {
        return line(
                path6Start,
                path6
        ).linear(
                path6Start,
                path6
        );
    }

    public Path path7() {
        return line(
                path6,
                path7
        ).linear(
                path6,
                path7
        );
    }
}