package org.firstinspires.ftc.teamcode.tata.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.Common.tataMecanumDrive;

@Autonomous(name="RED - Auto - Warehouse", group="RED")

public class AutoRedWarehouse extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(18, -63.5, Math.toRadians(90)); //change x

    @Override
    public void runOpMode() throws InterruptedException {
        Trajectory traj0, traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10;
        Trajectory traj_last;

        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) {
            stopThreads();
            return;
        }
        //This should be done only after Start
        int barCodeLoc = sensorDriver.getBarCodeRED();
        telemetry.addData("BarCode", " loc %2d",barCodeLoc);
        telemetry.update();

        //Fix for testing
        barCodeLoc = 2;

        traj0 = robot.trajectoryBuilder((startPose))
                .forward(10, tataMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        tataMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.followTrajectory(traj0);
        traj_last = traj0;

        if (barCodeLoc == 1) {
            traj1 = robot.trajectoryBuilder((traj_last.end()))
                    .lineToLinearHeading(new Pose2d(-12, -41.5, Math.toRadians(270)))
                    .build();
            robot.followTrajectory(traj1);
            traj_last = traj1;
            //sleep(1000);
            slideDriver.moveRobotSlideBy(5, 0);
            sleep(1000);

        } else if (barCodeLoc == 2) {
            traj1 = robot.trajectoryBuilder((traj_last.end()))
                    //.lineToLinearHeading(new Pose2d(-12, -43, Math.toRadians(270)))
                    .lineToSplineHeading(new Pose2d(-24, -24, Math.toRadians(180)))
                    .build();
            robot.followTrajectory(traj1);
            traj_last = traj1;
            sleep(4000);
            //slideDriver.moveRobotSlideBy(10, 0);
            //sleep(1000);
            //Check correct
            imuParams = imuDriver.getRobotImuParams();
            //if (Math.abs(imuParams.correctedHeading - 180) > 5)
            //apply correction
            robot.turn(Math.toRadians(-1*(imuParams.correctedHeading - 180)));

        } else {
            traj1 = robot.trajectoryBuilder(traj_last.end())
                    .lineToLinearHeading(new Pose2d(-12, -50, Math.toRadians(270)))
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

        /*
        //robot.turn(Math.toRadians(220));
        sleep(500);
        slideDriver.dropGameElement();
*/

        //move near Alliance Hub (-12, -24)
////        traj4 = robot.trajectoryBuilder(traj_last.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
////                //.lineToSplineHeading(new Pose2d(4, -48, Math.toRadians(300)))
////                .build();
////        robot.followTrajectory(traj4);

//        sleep(1000);
//        slideDriver.moveRobotSlideBy(20, 0);
//        //slideDriver.slideHW.motorSetRawSpeed(-0.95);
//        sleep(2000);
//        slideDriver.dropGameElement();
//        sleep(1000);
//        slideDriver.moveRobotSlideBy(-20, 0);
//        //slideDriver.slideHW.motorSetRawSpeed(0.95);
//        sleep(500);
//        //slideDriver.stop();


        //Extend the slide and drop the game element
        //retract the slide
        //inTakeDriver.toggleIntake(true);

        traj5 = robot.trajectoryBuilder(traj_last.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .lineToSplineHeading(new Pose2d(18, -62, Math.toRadians(10)))
                .build();
        robot.followTrajectory(traj5);

        sleep(1000);
//
//        if (barCodeLoc == 1) {
//            slideDriver.moveRobotSlideBy(-5, 0);
//        } else if (barCodeLoc == 2) {
//            slideDriver.moveRobotSlideBy(-10, 0);
//        } else {
//            slideDriver.moveRobotSlideBy(-18, 0);
//        }

        sleep(500);
        traj6 = robot.trajectoryBuilder(traj5.end())
                .forward(36)
                .build();
        robot.followTrajectory(traj6);
        sleep(500);
        //inTakeDriver.stop();
/*
         traj7 = robot.trajectoryBuilder(traj6.end())
                .back(28)
                .build();
        robot.followTrajectory(traj7);

        traj8 = robot.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(4, -48, Math.toRadians(310)))
                .build();
        robot.followTrajectory(traj8);


        //sleep(1000);
        slideDriver.moveRobotSlideBy(20, 0);
        //slideDriver.slideHW.motorSetRawSpeed(-0.95);
        //sleep(2000);
        slideDriver.dropGameElement();
        sleep(500);
        slideDriver.moveRobotSlideBy(-20, 0);
        //slideDriver.slideHW.motorSetRawSpeed(0.95);
        sleep(500);
        //slideDriver.stop();

        traj9 = robot.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(18, -62, Math.toRadians(10)))
                .build();
        robot.followTrajectory(traj9);

        traj10 = robot.trajectoryBuilder(traj9.end())
                .forward(28)
                .build();
        robot.followTrajectory(traj10);
        sleep(500);
        inTakeDriver.stop();

 */


        stopThreads();


    }
}
