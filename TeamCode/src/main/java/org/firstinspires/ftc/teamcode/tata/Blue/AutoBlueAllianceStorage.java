package org.firstinspires.ftc.teamcode.tata.Blue;

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

@Autonomous(name = "BLUE - Auto - AllianceStorage", group = "BLUE")
public class AutoBlueAllianceStorage extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-42.25, 63.5, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);

        int barCodeLoc = sensorDriver.getBarCodeBLUE();
        RobotSensorParams dsParams = new RobotSensorParams();

        //while (!isStopRequested() && !isStarted()) {
        //}

        waitForStart();


        if (isStopRequested()) {
            stopThreads();
            return;
        }

        //Move towards team marker
        TrajectorySequence identifyTeamMarker = getTrajectorySequenceBuilder ()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 15, DriveConstants.TRACK_WIDTH ) ) ) )
                .strafeLeft(1.5)
                .forward(12)
                .build();

        robot.followTrajectorySequence(identifyTeamMarker);
        PoseStorage.currentPose = robot.getPoseEstimate();

        sleep(1000);
        barCodeLoc = sensorDriver.getBarCodeBLUE();
        telemetry.addData("Started. Element position", barCodeLoc);
        telemetry.update();

        Pose2d pose = getTeamMarkerCoord(SideColor.Blue, StartPos.Storage, barCodeLoc);
        int lvl = barCodeLoc;
        TrajectorySequence pickTeamMarker = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                //    armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.COLLECT);
                })
                //.waitSeconds(1)
                .lineToSplineHeading(pose)
                .addTemporalMarker(() -> {
                    moveSlideToPos(lvl, SlideDirection.OUT);
                })
                .waitSeconds(1.0)

                .forward(4)
                .addTemporalMarker(() -> {
                  //  armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.SAVE);
                })
                .build();
        robot.followTrajectorySequence(pickTeamMarker);
        PoseStorage.currentPose = robot.getPoseEstimate();
        sleep(500);

        if (barCodeLoc == 1) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, 24, Math.toRadians(180)))
                    .back(11.5)
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

        sleep(1000);
        slideDriver.dropGameElement();

        TrajectorySequence moveToDropCarousel = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    moveSlideToPos(lvl, SlideDirection.IN);
                })

                .lineToLinearHeading(new Pose2d(-60, 45, Math.toRadians(90)))
                .build();
        robot.followTrajectorySequence(moveToDropCarousel);
        PoseStorage.currentPose = robot.getPoseEstimate();

        sleep(1000);
        dsParams = sensorDriver.getRobotSensorParams();

        telemetry.addData("Distance on Front %2f", dsParams.x_RF1);
        telemetry.addData("Distance on Right %2f", dsParams.x_LS);
        telemetry.update();
        RobotLog.ii("SHANK", "Duck Side Blue - RS %.2f, F %.2f", dsParams.x_RS, dsParams.x_LF1);

        TrajectorySequence moveToStartCarousel = getTrajectorySequenceBuilder()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 15, DriveConstants.TRACK_WIDTH ) ) ) )

                .strafeLeft( Math.min(dsParams.x_LS - 1, 6))
                .waitSeconds(0.2)
                .forward(Math.min(dsParams.x_RF - 8, 12))
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(true);
                })
                .waitSeconds(4)
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(false);
                })
                .lineToLinearHeading(new Pose2d(-65, 37.5, Math.toRadians(90)))//y was -37

                .build();
        robot.followTrajectorySequence(moveToStartCarousel);
        PoseStorage.currentPose = robot.getPoseEstimate();

        stopThreads();

    }
}
