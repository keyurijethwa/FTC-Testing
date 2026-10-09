package org.firstinspires.ftc.teamcode.Test;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorHuskyLens;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Intake_Balls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedropath_test.ShootTest;

import java.util.List;

@Autonomous(name = "BioBuzzAutoBlue2DistanceLimelightTest")
public class BioBuzzAutoBlue2DistanceLimelightTest
        extends LinearOpMode {

    private Follower follower;
    private Servo rgbLight;

    private static final double RGB_RED = 0.30;
    private static final double RGB_YELLOW = 0.35;
    private static final double RGB_GREEN = 0.50;
    private static final double RGB_BLUE = 0.55;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(80.0381, 8.4235, 90);
    private final Pose path1Start = poseFactory.of(80.0381, 8.4235, 270);
    private final Pose path1 = poseFactory.of(83.0381, 38.8348, 270);
    private final Pose point2 = poseFactory.of(96.1132, 8.4235, 270);
    private final Pose point3 = poseFactory.of(84.7377, 38.8348, 270);
    private final Pose point4Start = poseFactory.of(84.7377, 38.8348, 0);
    private final Pose point4 = poseFactory.of(132.9726, 89.0915, 0);
    private final Pose point4Control1 = poseFactory.of(129.6981, 52.9575, 0);
    private final Pose point5 = poseFactory.of(84.7377, 102.5972, 90);
    private final Pose point6 = poseFactory.of(132.6283, 36.5264, 90);
    private final Pose point6Control1 = poseFactory.of(130.0123, 80.1075, 90);

    private ShootTest sb;
    private Intake_Balls ib;
    private Limelight3A limelight;

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

                instant(() -> rgbGreen()),
                follow(follower, path1()),
                conditional(
                        ()->isDistanceHigh(),
                        distanceMore(),
                        shoot()

                ),
                follow(follower, path2()),

                instant(() -> rgbYellow()),
                instant(() -> ib.in(0.8)),
                waitMs(2000),
                instant(() -> ib.stop1()),

                instant(() -> rgbBlue()),
                follow(follower, path3()),

                instant(() -> rgbRed()),
                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),
                waitMs(2000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> rgbGreen()),
                follow(follower, path4()),

                instant(() -> rgbYellow()),
                instant(() -> ib.in(0.8)),
                waitMs(2000),
                instant(() -> ib.stop1()),

                instant(() -> rgbBlue()),
                follow(follower, path5()),

                instant(() -> rgbRed()),
                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),
                waitMs(2000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> rgbGreen()),
                follow(follower, path6()),

                instant(() -> rgbGreen())
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


    @Override
    public void runOpMode() {

        sb = new ShootTest(hardwareMap);
        ib = new Intake_Balls(hardwareMap);
        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        rgbLight = hardwareMap.get(Servo.class, "rgb");

        rgbLight.setPosition(RGB_BLUE);

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        schedule(autoRoutine());

        while (opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData(
                        "Current path distance remaining",
                        follower.distanceToEndpoint()
                );
                telemetry.addData(
                        "Path number",
                        follower.pathIndex()
                );
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(path1Start, path1)
                .linear(path1Start, path1);
    }
    public Path path11(){
        return line(point5,path1).linear(point5,path1);
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
        return curve(point4Start, point4Control1, point4)
                .linear(point4Start, point4);
    }

    public Path path5() {
        return line(point4, point5)
                .linear(point4, point5);
    }
    public Path path51(){
        return line(path1,point5).linear(path1,point5);
    }

    public Path path6() {
        return curve(point5, point6Control1, point6)
                .linear(point5, point6);
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