package org.firstinspires.ftc.teamcode.tata.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.tata.Common.PoseStorage;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.tata.RobotSlide.RobotSlideDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import java.util.*;

@Autonomous(name="MANAV_BLUE - Auto - Warehouse", group="BLUE")
@Disabled
public class AutoBlueWarehouseManav extends tataAutonomousBase {

    double wallPos = 63;

    //start position
    public Pose2d startPose = new Pose2d(6, 61, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);
        int barCodeLoc = 1;
        RobotSensorParams dsParams = new RobotSensorParams();

        while( !isStopRequested( ) && !isStarted( ) ) {
            barCodeLoc = sensorDriver.getBarCodeBLUE();
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
        int lvl = barCodeLoc;
        TrajectorySequence dropPreloadedGE = getTrajectorySequenceBuilder()
//            return drive.trajectorySequenceBuilder( new Pose2d( 0, 61, Math.toRadians( 270 ) ) )
                .forward(10)
                .addTemporalMarker( ( ) -> {
                    //robot.liftToShippingHubHeight( height );
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                    moveSlideToPos(lvl, SlideDirection.OUT);

                } )

                //(0,42) coordinate to drop element onto the hub
                .lineToSplineHeading( new Pose2d(0, 35, Math.toRadians(67.5)) )
                .addTemporalMarker( ( ) -> {
                    slideDriver.dropGameElement();
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.IN);
                    moveSlideToPos(lvl, SlideDirection.IN);
                } )
                .waitSeconds( 1 )
                .lineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ))
                .build();
        robot.followTrajectorySequence(dropPreloadedGE);
        PoseStorage.currentPose = robot.getPoseEstimate();

        //Correct Robot Orientation
        //imuParams = imuDriver.getRobotImuParams();

        //Measure distance from the right hand side wall
        dsParams = sensorDriver.getRobotSensorParams();
        //telemetry.addData("imu angle %2f", imuParams.correctedHeading);
        telemetry.addData("Distance on Front %2f", dsParams.x_LS);
        telemetry.update();
        sleep(500);

        double ch = 0.0;
        TrajectorySequence moveToWarehouse = getTrajectorySequenceBuilder()
                //.turn(-1 * Math.toRadians(imuParams.correctedHeading - 0))
                .addTemporalMarker( ( ) -> {
                    inTakeDriver.toggleIntake(true); //switch on intake
                } )

                .lineToConstantHeading( new Vector2d( 48, wallPos ) ) // 48
                .waitSeconds(0.5)
                .addTemporalMarker( ( ) -> {
                    inTakeDriver.toggleIntake(false); //switch OFF intake
                } )
                .lineToConstantHeading( new Vector2d( 12, wallPos ) )
                .build( );
        robot.followTrajectorySequence(moveToWarehouse);
        PoseStorage.currentPose = robot.getPoseEstimate();

        //Add error corrections for imu + distance from wall
        //Correct Robot Orientation
        //imuParams = imuDriver.getRobotImuParams();

        //Measure distance from the right hand side wall
        dsParams = sensorDriver.getRobotSensorParams();
        telemetry.addData("Distance on Front %2f", dsParams.x_LS);
        telemetry.update();

        sleep(500);

        TrajectorySequence dropGE1 = getTrajectorySequenceBuilder()
                //.turn(-1 * Math.toRadians(imuParams.correctedHeading - 0))

                // move to dump block 1 in the top layer
                .addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
                } )
                .lineToSplineHeading( new Pose2d(0, 35, Math.toRadians(67.5)) )
                //.lineToSplineHeading( new Pose2d(0, 42, Math.toRadians(67.5)) )
                .addTemporalMarker( ( ) -> {
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                    moveSlideToPos(lvl, SlideDirection.OUT);
                    slideDriver.dropGameElement();
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.IN);
                    moveSlideToPos(lvl, SlideDirection.IN);
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
                } )
                .waitSeconds( 0.8 )

                // move to grab block 2
                .setTangent( Math.toRadians( 90 ) )
                //.splineToSplineHeading( new Pose2d( 18/*49*/, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                .lineToSplineHeading( new Pose2d( 12/*49*/, wallPos, Math.toRadians( 0 ) ))

                .build( );
        robot.followTrajectorySequence(dropGE1);
        PoseStorage.currentPose = robot.getPoseEstimate();

        /*
        //in warehouse picking up first element
        TrajectorySequence intakeGE = getTrajectorySequenceBuilder()
                .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 ); //intake motors start
                } )

                .lineToConstantHeading( new Vector2d( 50, wallPos ) ) // 53
                .lineToConstantHeading( new Vector2d( 12, wallPos ) )

                .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 ); //stop intake motors
                } )
                .build();
        robot.followTrajectorySequence(intakeGE);

        //splining to hub to drop element
        TrajectorySequence dropGE2 = getTrajectorySequenceBuilder()
                // move to dump block 2 in the top layer
                .addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
                } )

                .lineToSplineHeading( new Pose2d(0, 42, Math.toRadians(67.5)) )
                .addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
                } )
                .waitSeconds( 0.8 )
                .build();
        robot.followTrajectorySequence(dropGE2); */

        //parking
        TrajectorySequence parkTraj = getTrajectorySequenceBuilder()
                .setTangent( Math.toRadians( 90 ) )
                //.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                .lineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ))
                .lineToSplineHeading( new Pose2d(48, wallPos, Math.toRadians(0)) )
                .build();
        robot.followTrajectorySequence(parkTraj);
        PoseStorage.currentPose = robot.getPoseEstimate();

        stopThreads();
    }

}




