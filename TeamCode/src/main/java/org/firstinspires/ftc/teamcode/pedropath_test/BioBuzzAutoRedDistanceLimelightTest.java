package org.firstinspires.ftc.teamcode.pedropath_test;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Intake_Balls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.test_robot;

import java.util.List;

@Autonomous(name = "BioBuzzAutoRedDistanceLimelightTest", group = "Autonomous")
public class BioBuzzAutoRedDistanceLimelightTest extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(58.6796, 133.6867, 90);

    private final Pose path1 =
            poseFactory.of(58.6796, 109.6867, 90);

    private final Pose point2 =
            poseFactory.of(46.5765, 133.6867, 90);

    private final Pose point3 =
            poseFactory.of(58.6796, 105.6867, 90);

    private final Pose point4Start =
            poseFactory.of(58.6796, 105.6867, 180);

    private final Pose point4Control1 =
            poseFactory.of(14.4915, 89.8642, 180);

    private final Pose point4 =
            poseFactory.of(12.6459, 47.7531, 180);

    private final Pose point5Start =
            poseFactory.of(12.6459, 47.7531, 270);

    private final Pose point5 =
            poseFactory.of(57.6796, 39.8173, 270);

    private final Pose point6 =
            poseFactory.of(12.5235, 115.5194, 270);

    private final Pose point6Control1 =
            poseFactory.of(1.8802, 50.3368, 270);

    private test_robot tr;
    private Intake_Balls ib;
    private ShootTest sb;
    private Limelight3A limelight;

    private Servo rgbLight;

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

                waitMs(4000),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - DISTANCE CHECK");
                }),

                conditional(
                        () -> isDistanceHigh(),
                        distanceMore(),
                        shoot()
                ),

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 2");
                }),

                follow(follower, path2()),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - INTAKE");
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
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOTING");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(3000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - PATH 6");
                }),

                follow(follower, path6()),

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - AUTO COMPLETE");
                })
        );
    }

    public Command distanceMore() {

        return sequential(
                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 51");
                }),

                follow(follower, path51()),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - SECOND DISTANCE CHECK");
                }),

                conditional(
                        () -> isDistanceHigh(),
                        condition2True(),
                        condition2False()
                )
        );
    }

    public Command condition2True() {

        return sequential(
                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: BLUE - PATH 11");
                }),

                follow(follower, path11()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOTING");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(3000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                )
        );
    }

    public Command condition2False() {

        return sequential(
                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOTING");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(3000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: BLUE - PATH 11");
                }),

                follow(follower, path11())
        );
    }

    public Command shoot() {

        return sequential(
                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOTING");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(3000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                )
        );
    }

    @Override
    public void runOpMode() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        sb = new ShootTest(hardwareMap);
        ib = new Intake_Balls(hardwareMap);

        rgbLight = hardwareMap.get(
                Servo.class,
                "rgb"
        );

        rgbRed();
        follower.setPose(start);
        follower.update();
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("BIOBUZZ RED AUTO");
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

            double distance = getLimelightDistance();

            telemetry.addData(
                    "Limelight Distance",
                    "%.2f in",
                    distance
            );

            telemetry.update();
        }
    }

    public Path path1() {

        return line(start, path1)
                .linear(start, path1);
    }

    public Path path11() {

        return line(point5, path1)
                .linear(point5, path1);
    }

    public Path path2() {

        return line(path1, point2)
                .linear(path1, point2);
    }

    public Path path3() {

        return line(point2, point3)
                .linear(point2, point3);
    }

    public Path path4() {

        return curve(
                point4Start,
                point4Control1,
                point4
        ).linear(point4Start, point4);
    }

    public Path path5() {

        return line(point5Start, point5)
                .linear(point5Start, point5);
    }

    public Path path51() {

        return line(path1, point5)
                .linear(path1, point5);
    }

    public Path path6() {
        return curve(
                point5,
                point6Control1,
                point6
        ).linear(point5, point6);
    }

    private double getLimelightDistance() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : tags) {

            if (tag.getFiducialId() == 30) {

                Pose3D targetPose =
                        tag.getTargetPoseCameraSpace();

                if (targetPose == null) {
                    return -1;
                }

                double x = targetPose.getPosition().x;
                double y = targetPose.getPosition().y;
                double z = targetPose.getPosition().z;

                return Math.sqrt(
                        x * x +
                                y * y +
                                z * z
                ) * 39.3701;
            }
        }

        return -1;
    }

    private boolean isDistanceHigh() {

        double distance = getLimelightDistance();

        telemetry.addData(
                "Limelight Distance",
                "%.2f in",
                distance
        );

        telemetry.update();

        return distance < 18.5;
    }
}