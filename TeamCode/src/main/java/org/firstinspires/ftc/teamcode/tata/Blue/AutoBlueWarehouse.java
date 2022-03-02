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

        /*TrajectorySequence identifyTeamMarker = getTrajectorySequenceBuilder ()
                .forward(7.5, tataMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        tataMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.followTrajectorySequence(identifyTeamMarker);

        barCodeLoc = sensorDriver.getBarCodeBLUE();
        telemetry.addData("Started. Element position", barCodeLoc);
        telemetry.update();
        sleep(1000); */

        int lvl = barCodeLoc;
        TrajectorySequence dropPreloadedGE = getTrajectorySequenceBuilder()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint( 80 ), new MecanumVelocityConstraint( 50, 14.1 ) ) ) )
//80 and 50
                // move to dump initial block in designated layer
                .addTemporalMarker( ( ) -> {
                    //robot.liftToShippingHubHeight( height );
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                    moveSlideToPos(lvl, SlideDirection.OUT);
                } )
                .setTangent( Math.toRadians( 270 ) )
                .splineToLinearHeading( new Pose2d( -4.6,41.3 , Math.toRadians(67.5) ), Math.toRadians( 300 ) )
                //.lineToSplineHeading( new Pose2d(-4.6, 40.5, Math.toRadians(67)) )
                .addTemporalMarker( ( ) -> {
                    slideDriver.dropGameElement();
                    moveSlideToPos(lvl, SlideDirection.IN);
                } )
                .waitSeconds( 1 )

                //Grab Block 1 from warehouse
                .setTangent( Math.toRadians( 90) )
                .splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 10) )
                //.lineToSplineHeading( new Pose2d(12, wallPos, Math.toRadians(0)) )

                .build();
        robot.followTrajectorySequence(dropPreloadedGE);
        PoseStorage.currentPose = robot.getPoseEstimate();

        double headingCorrection = correctOrientationUsingImu(0);

        //Get Block #1 from warehouse
        TrajectorySequence dropWarehouseGE1 = getTrajectorySequenceBuilder()

                .turn(Math.toRadians(headingCorrection))
                .waitSeconds(0.3)
                .strafeLeft(0.5)
                .addTemporalMarker( ( ) -> {
                   // inTakeDriver.toggleIntake(true);
                } )
                .forward(42)
               // .back(42)
                //.lineToConstantHeading( new Vector2d( 54, wallPos ) ) // 48
                //.lineToConstantHeading( new Vector2d( 12, wallPos+1 ) )
                .addTemporalMarker( ( ) -> {
                    //stop intake
                    //inTakeDriver.toggleIntake(true);
                } )
                .addTemporalMarker( ( ) -> {
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                   // moveSlideToPos(3, SlideDirection.OUT);
                } )
                //.setTangent( Math.toRadians( 200) )
                //.splineToLinearHeading( new Pose2d( -4.6,41.3 , Math.toRadians(67.5) ), Math.toRadians( 270 ) )
                //.lineToSplineHeading( new Pose2d(-4.6, 41, Math.toRadians(67.5)) )
                .addTemporalMarker( ( ) -> {
                    //slideDriver.dropGameElement();
                } )
                .waitSeconds( 0.8 )
                .addTemporalMarker( ( ) -> {
                    //robot.liftToShippingHubHeight( height );
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.IN);
                    //moveSlideToPos(3, SlideDirection.IN);
                } )
                //.setTangent( Math.toRadians( 90) )
                //.splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 10) )
                //.lineToSplineHeading( new Pose2d(12, wallPos, Math.toRadians(0)) )
                .build();
        robot.followTrajectorySequence(dropWarehouseGE1);
        PoseStorage.currentPose = robot.getPoseEstimate();

        headingCorrection = correctOrientationUsingImu(0);
        //Get Block 2
        /*TrajectorySequence dropWarehouseGE2 = getTrajectorySequenceBuilder()
                .turn(Math.toRadians(headingCorrection))
                .waitSeconds(0.3)
                .strafeLeft(0.5)
                .addTemporalMarker( ( ) -> {
                    inTakeDriver.toggleIntake(true);
                } )
                .lineToConstantHeading( new Vector2d( 54, wallPos ) ) // 48
                .strafeLeft(0.5)
                .lineToConstantHeading( new Vector2d( 12, wallPos+1 ) )
                .addTemporalMarker( ( ) -> {
                    //stop intake
                    inTakeDriver.toggleIntake(true);
                } )
                .addTemporalMarker( ( ) -> {
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                    moveSlideToPos(3, SlideDirection.OUT);
                } )
                .setTangent( Math.toRadians( 200) )
                .splineToLinearHeading( new Pose2d( -4.6,41.3 , Math.toRadians(67.5) ), Math.toRadians( 270 ) )
                .addTemporalMarker( ( ) -> {
                    slideDriver.dropGameElement();
                } )
                .waitSeconds( 0.8 )
                .addTemporalMarker( ( ) -> {
                    //robot.liftToShippingHubHeight( height );
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.IN);
                    moveSlideToPos(3, SlideDirection.IN);
                } )
                .setTangent( Math.toRadians( 90) )
                .splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 10) )
                .waitSeconds(0.2)
                .build();
        robot.followTrajectorySequence(dropWarehouseGE2);

        headingCorrection = correctOrientationUsingImu(0);

        /*old parking trajectory
        TrajectorySequence parkTraj = getTrajectorySequenceBuilder()
                .setTangent( Math.toRadians( 90 ) )
                //.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                .lineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ))
                .lineToSplineHeading( new Pose2d(48, wallPos, Math.toRadians(0)) )
                .build();
        robot.followTrajectorySequence(parkTraj);

        //Get Block 3 and Park
        TrajectorySequence dropWarehouseGE3 = getTrajectorySequenceBuilder()
                .turn(Math.toRadians(headingCorrection))
                .waitSeconds(0.3)
                .strafeLeft(0.5)
                .addTemporalMarker( ( ) -> {
                    inTakeDriver.toggleIntake(true);
                } )
                .lineToConstantHeading( new Vector2d( 54, wallPos ) ) // 48
                .addTemporalMarker( ( ) -> {
                    //stop intake
                    inTakeDriver.toggleIntake(true);
                } )
                .build();
        robot.followTrajectorySequence(dropWarehouseGE3);*/
    }
}