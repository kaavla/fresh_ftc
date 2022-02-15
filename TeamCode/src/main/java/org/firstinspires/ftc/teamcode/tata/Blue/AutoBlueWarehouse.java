package org.firstinspires.ftc.teamcode.tata.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.Common.tataMecanumDrive;

@Autonomous(group = "robot")
public class AutoBlueWarehouse extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(18, -63.5, Math.toRadians(90)); //change x

    @Override
    public void runOpMode() throws InterruptedException {
        Trajectory traj0, traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10;
        Trajectory traj_last;
//        init(hardwareMap);
//
//        robot.setPoseEstimate(startPose);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        //armDriver.initArmPos(0);
//
///*
//        sleep(1000);
//        slideDriver.moveRobotSlideBy(20, 0);
//        //slideDriver.slideHW.motorSetRawSpeed(-0.95);
//        sleep(2000);
//        slideDriver.dropGameElement();
//        sleep(2000);
//        slideDriver.moveRobotSlideBy(-20, 0);
//        //slideDriver.slideHW.motorSetRawSpeed(0.95);
//        sleep(3000);
//
// */
//
//        traj1 = robot.trajectoryBuilder(startPose)
//                .back(16)
//                .build();
//        robot.followTrajectory(traj1);
//        sleep(500);
//        params = sensorDriver.getRobotSensorParams();
//
//        int bar_code_pos = -1;
//
//        //Try 1st bar code  position
//        if (params.x_LR < 5) {
//            bar_code_pos = 3;
//            //pick up team marker
//        }
//        traj_last = traj1;
//        //try 2nd if bar code not yet found
//        if (bar_code_pos == -1) {
//            //Move to the left 2 inches
//            traj2 = robot.trajectoryBuilder(traj_last.end())
//                    .strafeRight(5)
//                    .build();
//            robot.followTrajectory(traj2);
//            traj_last = traj2;
//            params = sensorDriver.getRobotSensorParams();
//            sleep(500);
//            if (params.x_RR < 5) {
//                bar_code_pos = 2;
//                //pick up team marker
//            }
//        }
//
//        //try 3rd if bar code not yet found
//        if (bar_code_pos == -1) {
//            //Move to the left 2 inches
//            traj3 = robot.trajectoryBuilder(traj_last.end())
//                    .strafeRight(5)
//                    .build();
//            robot.followTrajectory(traj3);
//            traj_last = traj3;
//            params = sensorDriver.getRobotSensorParams();
//            bar_code_pos = 1;
//            //pick up team marker
//
//        }
//
//        telemetry.addData("FOUND", " Found Marker  %2d", bar_code_pos);
//        telemetry.update();
//        sleep(1000);
//

        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);

        waitForStart();


        if (isStopRequested()) {
            stopThreads();
            return;
        }
        int barCodeLoc = sensorDriver.getBarCodeBLUE();
        telemetry.addData("BarCode", " loc %2d",barCodeLoc);
        telemetry.update();
        sleep(1000);
        traj0 = robot.trajectoryBuilder((startPose))
                .forward(10, tataMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        tataMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.followTrajectory(traj0);

        if (barCodeLoc == 1) {
            traj1 = robot.trajectoryBuilder((traj0.end()))
                    .lineToLinearHeading(new Pose2d(-12, 41.5, Math.toRadians(90)))
                    .build();
            robot.followTrajectory(traj1);
            traj_last = traj1;
            //sleep(1000);
            slideDriver.moveRobotSlideBy(5, 0);
            sleep(1000);

        } else if (barCodeLoc == 2) {
            traj1 = robot.trajectoryBuilder((traj0.end()))
                    .lineToLinearHeading(new Pose2d(-12, 43, Math.toRadians(90)))
                    .build();
            robot.followTrajectory(traj1);
            traj_last = traj1;
            //sleep(1000);
            slideDriver.moveRobotSlideBy(10, 0);
            sleep(1000);

        } else {
            traj1 = robot.trajectoryBuilder(traj0.end())
                    .lineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)))
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
        //robot.turn(Math.toRadians(220));
        sleep(500);
        slideDriver.dropGameElement();


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

        traj5 = robot.trajectoryBuilder(traj_last.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                .lineToSplineHeading(new Pose2d(18, 62, Math.toRadians(100)))
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
