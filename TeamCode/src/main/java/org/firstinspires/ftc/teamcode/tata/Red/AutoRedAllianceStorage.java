package org.firstinspires.ftc.teamcode.tata.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.PoseStorage;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.Common.tataMecanumDrive;
import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.tata.RobotSlide.RobotSlideDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous(name = "RED - Auto - AllianceStorage", group = "RED")
public class AutoRedAllianceStorage extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-42.25, -63.5, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose, opModeCalled.AUTO);
        robot.setPoseEstimate(startPose);
        int barCodeLoc = sensorDriver.getBarCodeRED();
        RobotSensorParams dsParams = new RobotSensorParams();

        //while (!isStopRequested() && !isStarted()) {
           // barCodeLoc = sensorDriver.getBarCodeRED();
           // telemetry.addData("Waiting to Start. Element position", barCodeLoc);
           // telemetry.update();
        //}

        waitForStart();
      //  telemetry.addData("Started. Element position", barCodeLoc);
       // telemetry.update();

        if (isStopRequested()) {
            stopThreads();
            return;
        }
        //Move towards team marker
        TrajectorySequence identifyTeamMarker = getTrajectorySequenceBuilder ()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 15, DriveConstants.TRACK_WIDTH ) ) ) )
                .strafeRight(1.5)
                .forward(12)
                .build();
        robot.followTrajectorySequence(identifyTeamMarker);
        PoseStorage.currentPose = robot.getPoseEstimate();

        sleep(1000);

        barCodeLoc = sensorDriver.getBarCodeRED();
        telemetry.addData("Started. Element position", barCodeLoc);
        telemetry.update();

        Pose2d pose = getTeamMarkerCoord(SideColor.Red, StartPos.Storage, barCodeLoc);
        int lvl = barCodeLoc;
        TrajectorySequence pickTeamMarker = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    //Robot Arm to Collect Pos
                    //armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.COLLECT);
                })
                //.waitSeconds(1)
                .lineToSplineHeading(pose)
                .addTemporalMarker(() -> {
                    //slideDriver.moveRobotSlideBy(slideLen, 0);
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                    moveSlideToPos(lvl, SlideDirection.OUT);
                })
                .waitSeconds(1.0)

                .forward(4)
                .addTemporalMarker(() -> {
                    //armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.SAVE);
                })
                //.waitSeconds(1.0)
                .build();
        robot.followTrajectorySequence(pickTeamMarker);
        PoseStorage.currentPose = robot.getPoseEstimate();
        sleep(500);

        if (barCodeLoc == 1) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, -24, Math.toRadians(180)))
                    .back(11.5)
                    .build();
            robot.followTrajectorySequence(moveToDropGE);
        }
        if (barCodeLoc == 2) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, -24, Math.toRadians(180)))
                    .back(12)
                    .build();
            robot.followTrajectorySequence(moveToDropGE);
        }
        if (barCodeLoc == 3) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, -24, Math.toRadians(180)))
                    .back(11)
                    .build();
            robot.followTrajectorySequence(moveToDropGE);

        }

        sleep(1000);
        slideDriver.dropGameElement();

        TrajectorySequence moveToDropCarousel = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    //Draw Sides in
                    //slideDriver.moveRobotSlideBy(-1*slideLen, 0);
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.IN);
                    moveSlideToPos(lvl, SlideDirection.IN);
                })

                .lineToLinearHeading(new Pose2d(-60, -45, Math.toRadians(270)))
                .build();
        robot.followTrajectorySequence(moveToDropCarousel);
        PoseStorage.currentPose = robot.getPoseEstimate();

        //Correct Robot Orientation
        //imuParams = imuDriver.getRobotImuParams();
        //robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - 270));

        //Measure distance from the right hand side wall
        sleep(1000);
        dsParams = sensorDriver.getRobotSensorParams();

        telemetry.addData("Distance on Front %2f", dsParams.x_LF1);//was using LF
        telemetry.addData("Distance on Right %2f", dsParams.x_RS);
        telemetry.update();

        RobotLog.ii("SHANK", "Duck Side Red - RS %.2f, F %.2f", dsParams.x_RS, dsParams.x_LF1);


        TrajectorySequence moveToStartCarousel = getTrajectorySequenceBuilder()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 15, DriveConstants.TRACK_WIDTH ) ) ) )
                .strafeRight(Math.min(dsParams.x_RS - 1, 6))
                .waitSeconds(0.2)
                .forward(Math.min(dsParams.x_LF1 - 8, 12))  //Carousel if of radius 7.5 inch //was using LF
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(true);
                })
                .waitSeconds(4)
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(false);
                })
                .lineToLinearHeading(new Pose2d(-65, -37.3, Math.toRadians(270)))//y was -37

                .build();

        robot.followTrajectorySequence(moveToStartCarousel);
        PoseStorage.currentPose = robot.getPoseEstimate();

        stopThreads();

    }
}
