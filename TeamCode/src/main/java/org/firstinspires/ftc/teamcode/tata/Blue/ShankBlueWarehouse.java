package org.firstinspires.ftc.teamcode.tata.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.tata.RobotSlide.RobotSlideDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

//import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder


@Autonomous(name="SHANK BLUE - Auto - Warehouse", group="BLUE")
public class ShankBlueWarehouse extends tataAutonomousBase {

    double wallPos = 63;

    //start position
    public Pose2d startPose = new Pose2d(6, 61, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);

        int barCodeLoc = 3;
        RobotSensorParams dsParams = new RobotSensorParams();
        /*
        while( !isStopRequested( ) && !isStarted( ) ) {
            barCodeLoc = sensorDriver.getBarCodeBLUE();
            telemetry.addData( "Waiting to Start. Element position", barCodeLoc );
            telemetry.update();
        }
        */

        Pose2d dropPos = new Pose2d( -4.6,40 , Math.toRadians(82) );
        waitForStart();
        //telemetry.addData( "Started. Element position", barCodeLoc );
        //telemetry.update();

        if (isStopRequested()) {
            stopThreads();
            return;
        }
        int lvl = 3;
        TrajectorySequence dropPreloadedGE = getTrajectorySequenceBuilder()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint( 60 ), new MecanumVelocityConstraint( 53, 14.1 ) ) ) )

                // move to dump initial block in designated layer
                .addTemporalMarker( ( ) -> {
					//robot.liftToShippingHubHeight( height );
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                    //moveSlideToPos(lvl, SlideDirection.OUT);
                    custommoveSlideToPos(lvl, SlideDirection.OUT, 19.5, 0.0);
                } )
                .setTangent( Math.toRadians( 270 ) )
                .splineToLinearHeading( new Pose2d( -4.6,46 , Math.toRadians(82) ), Math.toRadians( 300 ) )
                .waitSeconds( 0.5 )
                .addTemporalMarker( ( ) -> {
                    slideDriver.dropGameElement();
                } )
                .waitSeconds( 1 )

                .addTemporalMarker( ( ) -> {
                    //moveSlideToPos(lvl, SlideDirection.IN);
                    custommoveSlideToPos(lvl, SlideDirection.IN, 19.5, 0.0);
                } )

                //Grab Block 1 Start ////////////////////////////
                .setTangent( Math.toRadians( 90) )
                .splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 10) )

                .build();
        robot.followTrajectorySequence(dropPreloadedGE);

        //double headingCorrection = correctOrientationUsingImu(0);

        //Get Block #1
        TrajectorySequence dropPreloadedGE1 = getTrajectorySequenceBuilder()
                //.turn(Math.toRadians(headingCorrection))
                //.waitSeconds(0.3)
                .strafeLeft(1.5)

                .addTemporalMarker( ( ) -> {
                    inTakeDriver.intakeSet(true, true);
                } )
                .forward(40)
                .waitSeconds(0.2)
                .addTemporalMarker( ( ) -> {
                    inTakeDriver.intakeSet(true, false);
                } )
                .waitSeconds(0.2)
                .back(40)
                .addTemporalMarker( ( ) -> {
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                    //moveSlideToPos(1, SlideDirection.OUT);
                    custommoveSlideToPos(lvl, SlideDirection.OUT, 7.5, 0.0);
                } )
                //.lineToConstantHeading( new Vector2d( 54, wallPos ) ) // 48
                //.lineToConstantHeading( new Vector2d( 12, wallPos+1 ) )
                .addTemporalMarker( ( ) -> {
                    //stop intake
                    inTakeDriver.intakeSet(false, false);


                } )
                .setTangent( Math.toRadians( 200) )

                .splineToLinearHeading( dropPos, Math.toRadians( 270 ) )
                .waitSeconds(0.5)
                .addTemporalMarker( ( ) -> {
                    slideDriver.dropGameElement();
                } )
                .waitSeconds( 1.2 )

                .addTemporalMarker( ( ) -> {
                    //robot.liftToShippingHubHeight( height );
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.IN);
                    custommoveSlideToPos(lvl, SlideDirection.IN, 7.5, 0.0);
                    //moveSlideToPos(1, SlideDirection.IN);
                } )

                .setTangent( Math.toRadians( 90) )
                .splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 10) )
                .build();
        robot.followTrajectorySequence(dropPreloadedGE1);


        //headingCorrection = correctOrientationUsingImu(0);
        //Get Block 2
        TrajectorySequence dropPreloadedGE2 = getTrajectorySequenceBuilder()
                //.turn(Math.toRadians(headingCorrection))
               // .waitSeconds(0.3)
                .strafeLeft(1)

                .addTemporalMarker( ( ) -> {
                    inTakeDriver.intakeSet(true, true);
                } )
                .forward(45)
                .waitSeconds(0.2)
                .addTemporalMarker( ( ) -> {
                    inTakeDriver.intakeSet(true, false);
                } )
                .back(25)
                //.lineToConstantHeading( new Vector2d( 54, wallPos ) ) // 48
                //.lineToConstantHeading( new Vector2d( 12, wallPos+1 ) )
                .addTemporalMarker( ( ) -> {
                    //stop intake
                    inTakeDriver.intakeSet(false, false);

                } )
                .strafeLeft(1.5)
                .back(27)

                .addTemporalMarker( ( ) -> {
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                    custommoveSlideToPos(lvl, SlideDirection.OUT, 7.5, 0.0);
                } )


                .setTangent( Math.toRadians( 200) )
                .splineToLinearHeading( dropPos, Math.toRadians( 270 ) )
                .waitSeconds(0.3)
                .addTemporalMarker( ( ) -> {
                    slideDriver.dropGameElement();
                } )
                .waitSeconds( 1.3 )
                /*
                .addTemporalMarker( ( ) -> {
                    //robot.liftToShippingHubHeight( height );
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.IN);
                    custommoveSlideToPos(lvl, SlideDirection.IN, 7, 0.0);
                    //moveSlideToPos(3, SlideDirection.IN);
                } )
                */

                //.forward(4)
                .setTangent( Math.toRadians( 90) )
                .splineToSplineHeading( new Pose2d( 42, wallPos+4, Math.toRadians( 0 ) ), Math.toRadians( 10) )
                //.waitSeconds(0.2)
                .forward(50)
                .build();
        robot.followTrajectorySequence(dropPreloadedGE2);

        //headingCorrection = correctOrientationUsingImu(0);
        //Get Block 3 and Park
        TrajectorySequence dropPreloadedGE3 = getTrajectorySequenceBuilder()
                //.turn(Math.toRadians(headingCorrection))
                //.waitSeconds(0.3)
                .strafeLeft(3)
               .addTemporalMarker( ( ) -> {
                    inTakeDriver.toggleIntake(true);
                } )
               // .lineToConstantHeading( new Vector2d( 54, wallPos ) ) // 48
                .forward(42)
                .addTemporalMarker( ( ) -> {
                    //stop intake
                    inTakeDriver.toggleIntake(true);
                } )
                .build();
        robot.followTrajectorySequence(dropPreloadedGE3);

        RobotLog.ii("SHANK", "Reached here...end");

    }

    }




