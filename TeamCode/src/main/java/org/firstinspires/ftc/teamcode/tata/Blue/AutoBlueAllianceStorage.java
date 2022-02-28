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
        int barCodeLoc = sensorDriver.getBarCodeBLUE();
        RobotSensorParams dsParams = new RobotSensorParams();

        while (!isStopRequested() && !isStarted()) {
        }

        waitForStart();


        if (isStopRequested()) {
            stopThreads();
            return;
        }

        TrajectorySequence identifyTeamMarker = getTrajectorySequenceBuilder ()
                .forward(7.5, tataMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                    armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.COLLECT);
                })
                .waitSeconds(1)
                .lineToSplineHeading(pose)
                .addTemporalMarker(() -> {
                    moveSlideToPos(lvl, SlideDirection.OUT);
                })
                .waitSeconds(1.0)

                .forward(4)
                .addTemporalMarker(() -> {
                    armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.SAVE);
                })
                .build();
        robot.followTrajectorySequence(pickTeamMarker);
        sleep(500);

        if (barCodeLoc == 3) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, 25, Math.toRadians(180)))
                    .back(12)
                    .build();
            robot.followTrajectorySequence(moveToDropGE);
        }
        if (barCodeLoc == 2) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, 25, Math.toRadians(180)))
                    .back(13)
                    .build();
            robot.followTrajectorySequence(moveToDropGE);
        }
        if (barCodeLoc == 1) {
            TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                    .lineToSplineHeading(new Pose2d(-44, 25, Math.toRadians(180)))
                    .back(15)
                    .build();
            robot.followTrajectorySequence(moveToDropGE);

        }

        sleep(1000);
        slideDriver.dropGameElement();

        TrajectorySequence moveToDropCarousel = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    moveSlideToPos(lvl, SlideDirection.IN);
                })

                .lineToLinearHeading(new Pose2d(-56, 43, Math.toRadians(90)))
                .build();
        robot.followTrajectorySequence(moveToDropCarousel);

        dsParams = sensorDriver.getRobotSensorParams();

        telemetry.addData("Distance on Front %2f", dsParams.x_RF1);
        telemetry.addData("Distance on Right %2f", dsParams.x_LS);
        telemetry.update();

        TrajectorySequence moveToStartCarousel = getTrajectorySequenceBuilder()
                .strafeLeft(dsParams.x_LS - 1)
                .waitSeconds(0.2)
                .forward(dsParams.x_RF - 8)
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(false);
                })
                .waitSeconds(4)
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(false);
                })
                .lineToLinearHeading(new Pose2d(-65, 34, Math.toRadians(90)))//y was -37

                .build();
        robot.followTrajectorySequence(moveToStartCarousel);

        stopThreads();

    }
}
