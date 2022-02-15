package org.firstinspires.ftc.teamcode.tata.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.Common.tataMecanumDrive;

@Autonomous(name="RED - Auto - AllianceStorage", group="RED")
public class AutoRedAllianceStorage extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-42.25, -63.5, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj0, traj1, traj2, traj3, traj4, traj5, traj6,traj6A, traj7, traj8;
        Trajectory traj_last;
        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);

        waitForStart();
        int barCodeLoc = sensorDriver.getBarCodeRED();
        telemetry.addData("BarCode", " loc %2d",barCodeLoc);
        telemetry.update();

        if (isStopRequested()) {
            stopThreads();
            return;
        }


        if (barCodeLoc == 1) {
            traj1 = robot.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-12, -41.5, Math.toRadians(270)))
                    .build();
            robot.followTrajectory(traj1);
            traj_last = traj1;
            //sleep(1000);
            slideDriver.moveRobotSlideBy(5, 0);
            sleep(1000);

        } else if (barCodeLoc == 2) {
            traj1 = robot.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-12, -43, Math.toRadians(270)))
                    .build();
            robot.followTrajectory(traj1);
            traj_last = traj1;
            //sleep(1000);
            slideDriver.moveRobotSlideBy(10, 0);
            sleep(1000);

        } else {
            traj1 = robot.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-12, -48, Math.toRadians(270)))
                    .build();
            robot.followTrajectory(traj1);
            //sleep(1000);
            slideDriver.moveRobotSlideBy(19, 0);
            sleep(2000);
            traj2 = robot.trajectoryBuilder((traj1.end()))
                    .back(3, tataMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            tataMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            robot.followTrajectory(traj2);
            traj_last = traj2;

        }

        sleep(500);
        slideDriver.dropGameElement();

        //go to carousel
        traj5 = robot.trajectoryBuilder(traj_last.end())
                .lineToLinearHeading(new Pose2d(-56, -55, Math.toRadians(270)))
                .build();
        robot.followTrajectory(traj5);

        if (barCodeLoc == 1) {
            slideDriver.moveRobotSlideBy(-5, 0);
        } else if (barCodeLoc == 2) {
            slideDriver.moveRobotSlideBy(-10, 0);
        } else {
            slideDriver.moveRobotSlideBy(-18, 0);
        }

        traj6 = robot.trajectoryBuilder(traj5.end())
                .strafeRight(7)
                .build();
        robot.followTrajectory(traj6);
        sleep(2000);

        crDriver.toggleCarousel(true);
        sleep(5000);
        crDriver.toggleCarousel(true);

//going to the red alliance storage square
        traj7 = robot.trajectoryBuilder(traj6.end())
                //.lineToSplineHeading(new Pose2d(-60, -38, Math.toRadians(270)))
                .back(17, tataMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                tataMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.followTrajectory(traj7);


        stopThreads();

    }
}
