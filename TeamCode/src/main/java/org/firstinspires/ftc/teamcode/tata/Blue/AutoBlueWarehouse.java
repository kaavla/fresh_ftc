package org.firstinspires.ftc.teamcode.tata.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
//import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import java.util.*;




@Autonomous(name="BLUE - Auto - Warehouse", group="BLUE")
public class AutoBlueWarehouse extends tataAutonomousBase {

    //prob change later idk
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

        TrajectorySequence doTheThings = getTrajectorySequenceBuilder()
//            return drive.trajectorySequenceBuilder( new Pose2d( 0, 61, Math.toRadians( 270 ) ) )
                    .forward(6)
                    .addTemporalMarker( ( ) -> {
                        //robot.liftToShippingHubHeight( height );
                    } )
                    .lineToSplineHeading( new Pose2d(0, 42, Math.toRadians(67.5)) )
                    .addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
                    } )

                    .waitSeconds( 0.8 )

                    // move to grab block 1
                    .setTangent( Math.toRadians( 90 ) )
                    //.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                    .lineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ))
                    .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
                    } )
                    .lineToConstantHeading( new Vector2d( 48, wallPos ) ) // 48
                    .lineToConstantHeading( new Vector2d( 12, wallPos ) )
                    .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
                    } )

                    // move to dump block 1 in the top layer
                    .addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
                    } )
                    //.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ), Math.toRadians( 270 ) )
                    //.lineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ) )
                    .lineToSplineHeading( new Pose2d(0, 42, Math.toRadians(67.5)) )
                    .addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
                    } )
                    .waitSeconds( 0.8 )

                    // move to grab block 2
                    .setTangent( Math.toRadians( 90 ) )
                    //.splineToSplineHeading( new Pose2d( 18/*49*/, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                    .lineToSplineHeading( new Pose2d( 12/*49*/, wallPos, Math.toRadians( 0 ) ))
                    .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
                    } )
                    .lineToConstantHeading( new Vector2d( 50, wallPos ) ) // 53
                    .lineToConstantHeading( new Vector2d( 12, wallPos ) )
                    .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
                    } )

                    // move to dump block 2 in the top layer
                    .addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
                    } )
                    //.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ), Math.toRadians( 270 ) )
                    //.lineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ))
                    .lineToSplineHeading( new Pose2d(0, 42, Math.toRadians(67.5)) )
                    .addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
                    } )
                    .waitSeconds( 0.8 )

                    // move to grab block 3
                    .setTangent( Math.toRadians( 90 ) )
                    //.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                    .lineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ))
                    .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
                    } )
                    .lineToConstantHeading( new Vector2d( 52, wallPos ) ) // 50
                    .lineToConstantHeading( new Vector2d( 12, wallPos ) )
                    .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
                    } )

                    // move to dump block 3 in the top layer
                    .addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
                    } )
                    //.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ), Math.toRadians( 270 ) )
                    //.lineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ))
                    .lineToSplineHeading( new Pose2d(0, 42, Math.toRadians(67.5)) )
                    .addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
                    } )
                    .addTemporalMarker( ( ) -> {
//					robot.drive.setDeadwheelsDisabledCheck( ( ) -> true );
//					robot.odometryLift.liftOdometry( );
                    } )
                    .waitSeconds( 0.8 )

                    // turn towards the
                    .turn( Math.toRadians( 110 ) )
                    /*// move to barrier to park
                    .setTangent( Math.toRadians( 90 ) )
                    .splineToSplineHeading( new Pose2d( 11.5, 44, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                    .lineToConstantHeading( new Vector2d( 62, 44 ) )*/
                    .build( );


        robot.followTrajectorySequence(doTheThings);


        }
       /* Pose2d pose = getTeamMarkerCoord(SideColor.Blue,StartPos.Warehouse, barCodeLoc);
        double slideLen = getSlideHeightByLvlInInch(barCodeLoc);

        TrajectorySequence pickTeamMarker = getTrajectorySequenceBuilder()
                .addTemporalMarker( ( ) -> {
                    //Robot Arm to Collect Pos
                    armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.COLLECT);
                    telemetry.addData("moving arm", "line51");
                } )
                .waitSeconds(3)
                .lineToSplineHeading(pose)
                .addTemporalMarker( ( ) -> {
                    slideDriver.moveRobotSlideBy(slideLen, 0);
                } )
                .waitSeconds(0.5)
//picking up team marker
                .forward(3)
                .addTemporalMarker( ( ) -> {
                    armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.SAVE);
                } )
                //.waitSeconds(1.0)

                .build();
        robot.followTrajectorySequence(pickTeamMarker);

//
        TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                .lineToSplineHeading(new Pose2d(11.5, 24, Math.toRadians(0)))
                .waitSeconds(5.0)
                .back(10)
                .build();
        robot.followTrajectorySequence(moveToDropGE);

        sleep(500);
        slideDriver.dropGameElement();

//        TrajectorySequence moveToDropCarousel = getTrajectorySequenceBuilder()
//                .addTemporalMarker( ( ) -> {
//                    //Draw Slides in
//                    slideDriver.moveRobotSlideBy(-1*slideLen, 0);
//
//                } )
//
//                .lineToLinearHeading(new Pose2d(-60, 45, Math.toRadians(90)))
//                .build();
//        robot.followTrajectorySequence(moveToDropCarousel);

        //Correct Robot Orientation
        imuParams = imuDriver.getRobotImuParams();
        robot.turn(-1*Math.toRadians(imuParams.correctedHeading - 90));

        //Measure distance from the right hand side wall
        dsParams = sensorDriver.getRobotSensorParams();

        telemetry.addData( "Distance on Front %2f", dsParams.x_RF );
        telemetry.addData( "Distance on Right %2f", dsParams.x_LS );
        telemetry.update();

        sleep(3000);

//        TrajectorySequence moveToStartCarousel = getTrajectorySequenceBuilder()
//                .strafeLeft(dsParams.x_LS - 1)
//                .waitSeconds(0.2)
//                .forward(dsParams.x_RF - 8)  //Carousel if of radius 7.5 inch
//                .addTemporalMarker( ( ) -> {
//                    //start Carosel motor
//                    crDriver.toggleCarousel(true);
//                } )
//                .waitSeconds(4)
//                .addTemporalMarker( ( ) -> {
//                    //start Carosel motor
//                    crDriver.toggleCarousel(true);
//                } )
//                .lineToLinearHeading(new Pose2d(0, 63.5, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(48, 63.5, Math.toRadians(0)))
//
//                .build();
//        robot.followTrajectorySequence(moveToStartCarousel);

        stopThreads(); */

    }




