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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "BioBuzzAutoPath8Blue", group = "Autonomous")
public class BioBuzzAutoPath8Blue extends LinearOpMode {

    private Follower follower;

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

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                waitMs(2000),
                follow(follower, path2()),
                waitMs(2000),
                follow(follower, path3()),
                waitMs(2000),
                follow(follower, path4()),
                waitMs(2000),
                follow(follower, path5()),
                waitMs(2000),
                follow(follower, path6())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
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
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return line(path1, point2).linear(path1, point2);
    }

    public Path path3() {
        return line(point2, point3).linear(point2,point3);
    }

    public Path path4() {
        return curve(point4Start, point4Control1, point4).linear(point4Start,point4);
    }

    public Path path5() {
        return line(point4, point5).linear(point4,point5);
    }

    public Path path6() {
        return curve(point5, point6Control1, point6).linear(point5,point6);
    }
}