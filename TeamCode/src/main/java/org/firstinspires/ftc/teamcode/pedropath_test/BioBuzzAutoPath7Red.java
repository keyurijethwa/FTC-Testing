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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Intake_Balls;
import org.firstinspires.ftc.teamcode.ShooterSubsystem1;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.test_robot;

import java.util.List;

@Autonomous(name = "BioBuzzAutoPath7Red", group = "Autonomous")
public class BioBuzzAutoPath7Red extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(58.6796, 133.6867, 90);
    private final Pose path1 = poseFactory.of(58.6796, 105.6867, 90);
    private final Pose point2 = poseFactory.of(46.5765, 133.6867, 90);
    private final Pose point3 = poseFactory.of(58.5031, 105.6867, 90);
    private final Pose point4Start = poseFactory.of(58.5031, 105.6867, 180);
    private final Pose point4Control1 = poseFactory.of(14.4915, 89.8642, 180);
    private final Pose point4 = poseFactory.of(12.6459, 47.7531, 180);
    private final Pose point5Start = poseFactory.of(12.6459, 47.7531, 270);
    private final Pose point5 = poseFactory.of(57.6796, 39.8173, 270);
    private final Pose point6 = poseFactory.of(12.5235, 115.5194, 270);
    private final Pose point6Control1 = poseFactory.of(1.8802, 50.3368, 270);

    private test_robot tr;
    private Intake_Balls ib;

    private ShootTest sb;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(2000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),
                follow(follower,path2()),
                instant(()->ib.in(0.8)),
                waitMs(2000),
                instant(()->ib.stop1()),
                follow(follower,path3()),
                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(2000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),
                follow(follower,path4()),
                instant(()->ib.in(0.8)),
                waitMs(2000),
                instant(()->ib.stop1()),
                follow(follower,path5()),
                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(2000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),
                follow(follower,path6())
        );
    }
    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        sb=new ShootTest(hardwareMap);
//        limelight=hardwareMap.get(Limelight3A.class,"limelight");
//        tr=new test_robot(hardwareMap);
        ib=new Intake_Balls(hardwareMap);
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
            telemetry.update();
        }
    }

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return line(path1, point2).linear(path1, point2);
    }

    public Path path3() {
        return line(point2, point3).linear(point2, point3);
    }

    public Path path4() {
        return curve(point4Start,point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return curve(point5,point6Control1, point6).linear(point5, point6);
    }

}