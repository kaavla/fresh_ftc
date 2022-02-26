package org.firstinspires.ftc.teamcode.tata.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.Common.tataMecanumDrive;
import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.tata.RobotSlide.RobotSlideDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BLUE - Auto - AllianceStorage", group = "BLUE")
public class AutoBlueAllianceStorage extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-42.25, 63.5, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj0, traj1, traj2, traj3, traj4, traj5, traj6, traj6A, traj7, traj8;
        Trajectory traj_last;
        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);

        while (!isStopRequested() && !isStarted()) {
            //barCodeLoc = 1;//sensorDriver.getBarCodeBLUE();
           // telemetry.addData("Waiting to Start. Element position", barCodeLoc);
            //telemetry.update();
        }

        waitForStart();
        int barCodeLoc = 1;//sensorDriver.getBarCodeBLUE();
        RobotSensorParams dsParams = new RobotSensorParams();

        telemetry.addData("Started. Element position", barCodeLoc);
        telemetry.update();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        //double slideLen = getSlideHeightByLvlInInch(barCodeLoc);
        TrajectorySequence identifyTeamMarker = getTrajectorySequenceBuilder ()
                .forward(12, tataMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        tataMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.followTrajectorySequence(identifyTeamMarker);

        barCodeLoc = sensorDriver.getBarCodeBLUE();
        telemetry.addData("Started. Element position", barCodeLoc);
        telemetry.update();
        sleep(1000);

        Pose2d pose = getTeamMarkerCoord(SideColor.Blue, StartPos.Storage, barCodeLoc);
        int lvl = barCodeLoc;

        TrajectorySequence pickTeamMarker = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    //Robot Arm to Collect Pos
                    armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.COLLECT);
                })
                .waitSeconds(1)
                .lineToSplineHeading(pose)
                .addTemporalMarker(() -> {
                    //slideDriver.moveRobotSlideBy(slideLen, 0);
                    slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                })
                .waitSeconds(1.0)

                .forward(3)
                .addTemporalMarker(() -> {
                    armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.SAVE);
                })
                //.waitSeconds(1.0)
                .build();
        robot.followTrajectorySequence(pickTeamMarker);
        sleep(500);

        if (barCodeLoc == 1) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, 26, Math.toRadians(180)))
                    .back(16)
                    .build();
            robot.followTrajectorySequence(moveToDropGE);
        }
        if (barCodeLoc == 2) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, 24, Math.toRadians(180)))
                    .back(12)
                    .build();
            robot.followTrajectorySequence(moveToDropGE);
        }
        if (barCodeLoc == 3) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, 24, Math.toRadians(180)))
                    .back(11)
                    .build();
            robot.followTrajectorySequence(moveToDropGE);

        }

        sleep(500);
        slideDriver.dropGameElement();

        TrajectorySequence moveToDropCarousel = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    //Draw Sides in
                    //slideDriver.moveRobotSlideBy(-1*slideLen, 0);
                    slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.IN);
                })

                .lineToLinearHeading(new Pose2d(-60, 49, Math.toRadians(90)))
                .build();
        robot.followTrajectorySequence(moveToDropCarousel);

        //Correct Robot Orientation
        imuParams = imuDriver.getRobotImuParams();
        robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - 90));

        //Measure distance from the right hand side wall
        dsParams = sensorDriver.getRobotSensorParams();

        telemetry.addData("Distance on Front %2f", dsParams.x_RF);
        telemetry.addData("Distance on Right %2f", dsParams.x_LS);
        telemetry.update();

        //sleep(3000);

        TrajectorySequence moveToStartCarousel = getTrajectorySequenceBuilder()
                .strafeLeft(dsParams.x_LS - 1)
                .waitSeconds(0.2)
                .forward(dsParams.x_RF - 8)  //Carousel if of radius 7.5 inch
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(false);
                })
                .waitSeconds(4)
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(false);
                })
                .lineToLinearHeading(new Pose2d(-65, 33, Math.toRadians(90)))

                .build();
        robot.followTrajectorySequence(moveToStartCarousel);

        stopThreads();

    }
}
