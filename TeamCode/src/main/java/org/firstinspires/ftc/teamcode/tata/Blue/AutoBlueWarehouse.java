package org.firstinspires.ftc.teamcode.tata.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.PoseStorage;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.Common.tataMecanumDrive;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.tata.RobotSlide.RobotSlideDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.util.RobotLog;
//import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import java.util.*;

@Autonomous(name="BLUE - Auto - Warehouse", group="BLUE")
public class AutoBlueWarehouse extends tataAutonomousBase {

    double wallPos = 63;

    //start position
    public Pose2d startPose = new Pose2d(6, 61, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose, opModeCalled.AUTO);
        robot.setPoseEstimate(startPose);

        int barCodeLoc = 3;
        RobotSensorParams dsParams = new RobotSensorParams();

        waitForStart();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        //Move towards team marker
        TrajectorySequence identifyTeamMarker = getTrajectorySequenceBuilder ()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 15, DriveConstants.TRACK_WIDTH ) ) ) )
                .lineToLinearHeading(new Pose2d(8, 48, Math.toRadians(270)))
                //.strafeLeft(2)
                //.forward(12)
                .build();

        robot.followTrajectorySequence(identifyTeamMarker);
        PoseStorage.currentPose = robot.getPoseEstimate();

        sleep(500);
        barCodeLoc = sensorDriver.getBarCodeBLUE();
        telemetry.addData("Started. Element position", barCodeLoc);
        telemetry.update();
        //barCodeLoc = 3;
        int lvl = barCodeLoc;

        //Pose2d dropPos = new Pose2d( -4.6,38 , Math.toRadians(82) );
        Pose2d dropPos =new Pose2d( -0.4,51.5 , Math.toRadians(67.5));
        if (barCodeLoc == 1) {
            TrajectorySequence dropPreloadedGE = getTrajectorySequenceBuilder()
                    .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 40, DriveConstants.TRACK_WIDTH ) ) ) )

                    .addTemporalMarker(() -> {
                        moveSlideToPos(lvl, SlideDirection.OUT);
                    })
                    .setTangent(Math.toRadians(90 + 45))
                    .splineToLinearHeading(dropPos, Math.toRadians(180+67.5))
                    .back(12)
                    .waitSeconds(0.5)
                    .addTemporalMarker( ( ) -> {
                        slideDriver.dropGameElement();
                    } )
                    .waitSeconds( 1 )
                    .addTemporalMarker( ( ) -> {
                        moveSlideToPos(lvl, SlideDirection.IN);
                    } )
                    .build();
            robot.followTrajectorySequence(dropPreloadedGE);
        }

        if (barCodeLoc == 2) {
            TrajectorySequence dropPreloadedGE = getTrajectorySequenceBuilder()
                    .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 40, DriveConstants.TRACK_WIDTH ) ) ) )
                    .addTemporalMarker(() -> {
                        //moveSlideToPos(lvl, SlideDirection.OUT);
                        //custommoveSlideToPos(lvl,SlideDirection.OUT, 10.5,0.0);
                    })
                    .setTangent(Math.toRadians(90+45))
                    .splineToLinearHeading(dropPos, Math.toRadians(270))
                    .back(9.5)
                    .waitSeconds(0.5)
                    .addTemporalMarker( ( ) -> {
                        slideDriver.dropGameElement();
                    } )
                    .waitSeconds( 1 )
                    .addTemporalMarker( ( ) -> {
                        //moveSlideToPos(lvl, SlideDirection.IN);
                        //custommoveSlideToPos(lvl,SlideDirection.IN, 10.5, 0.1);
                    } )
                    .build();
            robot.followTrajectorySequence(dropPreloadedGE);
        }

        if (barCodeLoc == 3) {
            TrajectorySequence dropPreloadedGE = getTrajectorySequenceBuilder()
                    .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 40, DriveConstants.TRACK_WIDTH ) ) ) )
                    .addTemporalMarker(() -> {
                        moveSlideToPos(lvl, SlideDirection.OUT);
                    })
                    .setTangent(Math.toRadians(90+45))
                    .splineToLinearHeading(dropPos, Math.toRadians(270))
                    .back(8.5)
                    .waitSeconds(0.5)
                    .addTemporalMarker( ( ) -> {
                        slideDriver.dropGameElement();
                    } )
                    .waitSeconds( 1 )
                    .addTemporalMarker( ( ) -> {
                        moveSlideToPos(lvl, SlideDirection.IN);
                    } )
                    .build();
            robot.followTrajectorySequence(dropPreloadedGE);
        }
        PoseStorage.currentPose = robot.getPoseEstimate();

        TrajectorySequence dropPreloadedGE = getTrajectorySequenceBuilder()
                //.setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint( 60 ), new MecanumVelocityConstraint( 53, 14.1 ) ) ) )
                //Grab Block 1 Start ////////////////////////////
                .setTangent( Math.toRadians( 90) )
                .splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 10) )
                .build();
        robot.followTrajectorySequence(dropPreloadedGE);
        PoseStorage.currentPose = robot.getPoseEstimate();

        double headingCorrection = correctOrientationUsingImu(0);

        //Get Block #1
        TrajectorySequence dropPreloadedGE1 = getTrajectorySequenceBuilder()
                .turn(Math.toRadians(headingCorrection))
                //.waitSeconds(0.3)
                .strafeLeft(1)

                .addTemporalMarker( ( ) -> {
                    inTakeDriver.intakeSet(true, true);
                } )
                .forward(45)
                //.waitSeconds(0.2)
                .addTemporalMarker( ( ) -> {
                    inTakeDriver.intakeSet(true, false);
                } )

                //.back(12)
                //.strafeLeft(2)
                .back(45)
                .addTemporalMarker( ( ) -> {
                    //stop intake
                    inTakeDriver.intakeSet(false, false);
                } )
                .waitSeconds(0.2)
                .addTemporalMarker( ( ) -> {
                    //custommoveSlideToPos(lvl, SlideDirection.OUT, 5.0, 0.0);
                    moveSlideToPos(1, SlideDirection.OUT);

                } )
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d( 2.4, 51.5, Math.toRadians( 67.5 )), Math.toRadians(270-67.5))
                .back(14)
                .waitSeconds(0.5)
                .addTemporalMarker( ( ) -> {
                    slideDriver.dropGameElement();
                } )
                .waitSeconds( 1 )
                .addTemporalMarker( ( ) -> {
                    moveSlideToPos(1, SlideDirection.IN);
                } )

                .setTangent( Math.toRadians( 90) )
                .splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 10) )
                .build();
        robot.followTrajectorySequence(dropPreloadedGE1);
        PoseStorage.currentPose = robot.getPoseEstimate();
        headingCorrection = correctOrientationUsingImu(0);

        //Park
        TrajectorySequence park = getTrajectorySequenceBuilder()
                .turn(Math.toRadians(headingCorrection))
                //.waitSeconds(0.3)
                .strafeLeft(1)
                .forward(30)
                .build();
        robot.followTrajectorySequence(park);
        PoseStorage.currentPose = robot.getPoseEstimate();


        stopThreads();
    }
}
