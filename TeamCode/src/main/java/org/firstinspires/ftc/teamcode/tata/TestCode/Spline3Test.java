package org.firstinspires.ftc.teamcode.tata.TestCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;

@Autonomous(name = "RED - Spline3 - AllianceStorage", group = "RED")
@Disabled
public class Spline3Test extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose);

        waitForStart();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        Trajectory traj = robot.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(50, 20), 0)
                .build();

        //sleep(1000);
        slideDriver.moveRobotSlideBy(20, 0);
        //sleep(1000);


        robot.followTrajectory(traj);

        sleep(2000);

        Trajectory traj1 = robot.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(0, 0),  Math.toRadians(180))
                .build();

        slideDriver.moveRobotSlideBy(-20, 0);
        robot.followTrajectory(traj1);

        sleep(6000);

        stopThreads();

    }
}
