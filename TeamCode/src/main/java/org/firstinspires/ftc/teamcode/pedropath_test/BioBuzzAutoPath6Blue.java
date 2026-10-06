package org.firstinspires.ftc.teamcode.pedropath_test;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
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

import org.firstinspires.ftc.teamcode.Intake_Balls;
import org.firstinspires.ftc.teamcode.ShootBalls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.servo_d;

@Autonomous(name = "BioBuzzAutoPath6Blue")
public class BioBuzzAutoPath6Blue extends LinearOpMode {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.7079, 134.7056, 90);
    private final Pose path1Start = poseFactory.of(82.7079, 134.7056, 90);
    private final Pose path1 = poseFactory.of(83.2981, 102.2404, 90);
    private final Pose point2 = poseFactory.of(131.6217, 133.2102, 90);
    private final Pose point3 = poseFactory.of(83.5764, 101.9887, 270);
    private final Pose point4 = poseFactory.of(134.1689, 94.1453, 0);
    private final Pose point5 = poseFactory.of(82.9906, 37.1123, 90);
    private final Pose point5Control1 = poseFactory.of(130.0198, 56.5867, 0);
    private final Pose point6 = poseFactory.of(135.3566, 36.5009, 90);
    private Intake_Balls ib;
    private ShootBalls sb;
    private servo_d s;


    public Command autoRoutine() {
        return sequential(
                follow(follower,path1()),
                follow(follower, path2()),
                waitMs(1000),
                follow(follower,path3()),
                follow(follower,path4()),
                waitMs(1000),
                follow(follower,path5()),
                waitMs(1000),
                follow(follower,path6())
        );
    }
    @Override
    public void runOpMode() throws InterruptedException {
        ib=new Intake_Balls(hardwareMap);
        sb=new ShootBalls(hardwareMap);
        s=new servo_d(hardwareMap);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        s.setPX();
        s.setHalf();
        telemetry.addData("Stopper set",true);
        telemetry.update();
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
        return line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return line(point3, point4).reverseTangent();
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).reverseTangent();
    }

    public Path path6() {
        return line(point5, point6).reverseTangent();
    }

}
