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

@Autonomous(name = "BioBuzzAutoPath12Blue", group = "Autonomous")
public class BioBuzzAutoPath12Blue extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(81.4855, 6.2877, 90);
    private final Pose path1Start = poseFactory.of(81.4855, 6.2877, 270);

    private final Pose path1 = poseFactory.of(84.4292, 37.3008, 270);
    private final Pose point2 = poseFactory.of(95.5792, 7.9961, 270);
    private final Pose point3 = poseFactory.of(84.7802, 37.4283, 270);
    private final Pose point4 = poseFactory.of(132.9726, 91.4613, 0);
    private final Pose point4Control1 = poseFactory.of(133.0991, 51.084, 0);
    private final Pose point5 = poseFactory.of(84.2623, 106.267, 90);
    private final Pose point6 = poseFactory.of(130.3679, 130.5906, 90);
    private final Pose point7 = poseFactory.of(84.3821, 105.6792, 90);
    private final Pose point8 = poseFactory.of(132.0283, 40.3113, 90);
    private final Pose point8Control1 = poseFactory.of(131.9255, 77.8245, 90);
    private test_robot tr;
    private Intake_Balls ib;

    private ShootTest sb;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(3000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),
                follow(follower,path2()),
                instant(()->ib.in(0.8)),
                waitMs(3000),
                instant(()->ib.stop1()),
                follow(follower,path3()),
                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(3000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),
                follow(follower,path4()),
                instant(()->ib.in(0.8)),
                waitMs(3000),
                instant(()->ib.stop1()),
                follow(follower,path5()),
                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(3000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),
                follow(follower,path6()),
                instant(()->ib.in(0.8)),
                waitMs(3000),
                instant(()->ib.stop1()),
                follow(follower,path7()),
                parallel(instant(()->sb.forwardsh()),instant(()->ib.in(0.8))),
                waitMs(3000),
                parallel(instant(()->sb.stopsh()),instant(()->ib.stop1())),
                follow(follower,path8())

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
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return line(path1, point2).linear(path1, point2);
    }

    public Path path3() {
        return line(point2, point3).linear(point2, point3);
    }

    public Path path4() {
        return curve(point3,point4Control1, point4).linear(point3, point4);
    }

    public Path path5() {
        return line(point4, point5).linear(point4, point5);
    }
    public Path path6() {
        return line(point5, point6).linear(point5, point6);
    }
    public Path path7() {
        return line(point6, point7).linear(point6, point7);
    }

    public Path path8() {
        return curve(point7,point8Control1, point8).linear(point7, point8);
    }


}