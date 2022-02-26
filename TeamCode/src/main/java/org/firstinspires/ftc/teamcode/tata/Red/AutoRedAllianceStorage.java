package org.firstinspires.ftc.teamcode.tata.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.Common.tataMecanumDrive;
import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RED - Auto - AllianceStorage", group="RED")
public class AutoRedAllianceStorage extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-42.25, -63.5, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj0, traj1, traj2, traj3, traj4, traj5, traj6,traj6A, traj7, traj8;
        Trajectory traj_last;
        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);
        int barCodeLoc = 1;
        RobotSensorParams dsParams = new RobotSensorParams();

        while( !isStopRequested( ) && !isStarted( ) ) {
            barCodeLoc = sensorDriver.getBarCodeRED();
            telemetry.addData( "Waiting to Start. Element position", barCodeLoc );
            telemetry.update();
        }


        waitForStart();
        telemetry.addData( "Started. Element position", barCodeLoc );
        telemetry.update();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        Pose2d pose = getTeamMarkerCoord(SideColor.Red,StartPos.Storage, barCodeLoc);
        double slideLen = getSlideHeightByLvlInInch(barCodeLoc);

        TrajectorySequence pickTeamMarker = getTrajectorySequenceBuilder()
                .addTemporalMarker( ( ) -> {
                    //Robot Arm to Collect Pos
                    armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.COLLECT);
                } )
                .waitSeconds(1)
                .lineToSplineHeading(pose)
                .addTemporalMarker( ( ) -> {
                    slideDriver.moveRobotSlideBy(slideLen, 0);
                } )
                .waitSeconds(0.5)

                .forward(3)
                .addTemporalMarker( ( ) -> {
                    armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.SAVE);
                } )
                //.waitSeconds(1.0)

                .build();
        robot.followTrajectorySequence(pickTeamMarker);


        TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                .lineToSplineHeading(new Pose2d(-44, -24, Math.toRadians(180)))
                .back(10)
                .build();
        robot.followTrajectorySequence(moveToDropGE);

        sleep(500);
        slideDriver.dropGameElement();

        TrajectorySequence moveToDropCarousel = getTrajectorySequenceBuilder()
                .addTemporalMarker( ( ) -> {
                    //Draw Sides in
                    slideDriver.moveRobotSlideBy(-1*slideLen, 0);

                } )

                .lineToLinearHeading(new Pose2d(-60, -45, Math.toRadians(270)))
                .build();
        robot.followTrajectorySequence(moveToDropCarousel);

        //Correct Robot Orientation
        imuParams = imuDriver.getRobotImuParams();
        robot.turn(-1*Math.toRadians(imuParams.correctedHeading - 270));

        //Measure distance from the right hand side wall
        dsParams = sensorDriver.getRobotSensorParams();

        telemetry.addData( "Distance on Front %2f", dsParams.x_LF );
        telemetry.addData( "Distance on Right %2f", dsParams.x_RS );
        telemetry.update();

        sleep(3000);

        TrajectorySequence moveToStartCarousel = getTrajectorySequenceBuilder()
                .strafeRight(dsParams.x_RS - 1)
                .waitSeconds(0.2)
                .forward(dsParams.x_LF - 8)  //Carousel if of radius 7.5 inch
                .addTemporalMarker( ( ) -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(true);
                } )
                .waitSeconds(4)
                .addTemporalMarker( ( ) -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(true);
                } )
                .lineToLinearHeading(new Pose2d(-65, -37, Math.toRadians(270)))

                .build();
        robot.followTrajectorySequence(moveToStartCarousel);

        stopThreads();

    }
}
