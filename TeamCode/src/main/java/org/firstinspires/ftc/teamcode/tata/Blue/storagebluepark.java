package org.firstinspires.ftc.teamcode.tata.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;

@Autonomous(group = "robot")
public class storagebluepark extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-39.5, 66.25, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj0;
        init(hardwareMap, startPose, opModeCalled.AUTO);
        robot.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        //go forward

        traj0 = robot.trajectoryBuilder((startPose))
                .lineToSplineHeading(new Pose2d(-60, 35, Math.toRadians(90)))
                .build();
        robot.followTrajectory(traj0);


    }
}