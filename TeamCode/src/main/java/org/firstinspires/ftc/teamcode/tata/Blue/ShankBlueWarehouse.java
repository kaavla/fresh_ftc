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
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint( 80 ), new MecanumVelocityConstraint( 50, 14.1 ) ) ) )

                // move to dump initial block in designated layer
                .addTemporalMarker( ( ) -> {
					//robot.liftToShippingHubHeight( height );
                    //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
                    moveSlideToPos(lvl, SlideDirection.OUT);
                } )
                .setTangent( Math.toRadians( 270 ) )
                .splineToLinearHeading( new Pose2d( -4.6,41.3 , Math.toRadians(67.5) ), Math.toRadians( 300 ) )
                .addTemporalMarker( ( ) -> {
                    slideDriver.dropGameElement();
                } )
                .waitSeconds( 0.8 )

                .addTemporalMarker( ( ) -> {
                    moveSlideToPos(lvl, SlideDirection.IN);
                } )

                //Grab Block 1 Start ////////////////////////////
                .setTangent( Math.toRadians( 90) )
                .splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 10) )

                .build();
        robot.followTrajectorySequence(dropPreloadedGE);

        double headingCorrection = correctOrientationUsingImu(0);

        //Get Block #1
        TrajectorySequence dropPreloadedGE1 = getTrajectorySequenceBuilder()
                .turn(Math.toRadians(headingCorrection))
                .waitSeconds(0.3)
                .strafeLeft(1)

                .addTemporalMarker( ( ) -> {
                    inTakeDriver.toggleIntake(true);
                } )
                .forward(42)
                .back(42)
                //.lineToConstantHeading( new Vector2d( 54, wallPos ) ) // 48
                //.lineToConstantHeading( new Vector2d( 12, wallPos+1 ) )
                .addTemporalMarker( ( ) -> {
                    //stop intake
                    inTakeDriver.toggleIntake(false);
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
                .build();
        robot.followTrajectorySequence(dropPreloadedGE1);


        headingCorrection = correctOrientationUsingImu(0);
        //Get Block 2
        TrajectorySequence dropPreloadedGE2 = getTrajectorySequenceBuilder()
                .turn(Math.toRadians(headingCorrection))
                .waitSeconds(0.3)
                .strafeLeft(0.5)

                .addTemporalMarker( ( ) -> {
                    inTakeDriver.toggleIntake(true);
                } )
                .lineToConstantHeading( new Vector2d( 54, wallPos ) ) // 48
                .strafeLeft(1)
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
        robot.followTrajectorySequence(dropPreloadedGE2);

        headingCorrection = correctOrientationUsingImu(0);
        //Get Block 3 and Park
        TrajectorySequence dropPreloadedGE3 = getTrajectorySequenceBuilder()
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
        robot.followTrajectorySequence(dropPreloadedGE3);

        RobotLog.ii("SHANK", "Reached here...end");

    }

    }




