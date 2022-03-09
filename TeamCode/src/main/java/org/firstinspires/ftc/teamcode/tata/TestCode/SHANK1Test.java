package org.firstinspires.ftc.teamcode.tata.TestCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.tata.Common.Utils;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RED - SHANK1 - AllianceStorage", group = "RED")
@Disabled
public class SHANK1Test extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(5.75, 61, Math.toRadians(270)); //change x

    double wallPos = 63;

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj0, traj1, traj2, traj3, traj4, traj5, traj6, traj6A, traj7, traj8;
        Trajectory traj_last;
        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);

        waitForStart();
        int barCodeLoc = sensorDriver.getBarCodeRED();
        telemetry.addData("BarCode", " loc %2d", barCodeLoc);
        telemetry.update();

        barCodeLoc = 2;

        if (isStopRequested()) {
            stopThreads();
            return;
        }
        TrajectorySequence mainTrajectory = robot.trajectorySequenceBuilder(startPose)
                //.setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint( 90 ), new MecanumVelocityConstraint( 60, 17 ) ) ) )

                // move to dump initial block in designated layer
                .addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( height );
                } )
                .setTangent( Math.toRadians( 180 ) )
                //.splineToLinearHeading( MeepMeepPath.getHubPosition( -22.5, 270, 1.5, true ), Math.toRadians( 270 - 22.5 ) )
                .lineToSplineHeading( Utils.getHubPosition( -22.5, 270, 1.5, true ) )
                .addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
                } )

                .waitSeconds( 0.8 )

                // move to grab block 1
                .setTangent( Math.toRadians( 90 ) )
                //.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                .lineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 180 ) ))
                .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
                } )
                //.lineToConstantHeading( new Vector2d( 48, wallPos ) ) // 48
                //.lineToConstantHeading( new Vector2d( 12, wallPos ) )
                .forward(40)
                .back(40)
                .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
                } )

                // move to dump block 1 in the top layer
                .addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
                } )
                //.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ), Math.toRadians( 270 ) )
                .lineToSplineHeading( Utils.getHubPosition( -22.5, 270, 7, true ) )
                .addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
                } )
                .waitSeconds( 0.8 )

                // move to grab block 2
                .setTangent( Math.toRadians( 90 ) )
                //.splineToSplineHeading( new Pose2d( 18/*49*/, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                .lineToSplineHeading( new Pose2d( 12/*49*/, wallPos, Math.toRadians( 180 ) ))
                .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
                } )
                //.lineToConstantHeading( new Vector2d( 50, wallPos ) ) // 53
                //.lineToConstantHeading( new Vector2d( 12, wallPos ) )
                .forward(40)
                .back(40)
                .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
                } )

                // move to dump block 2 in the top layer
                .addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
                } )
                //.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ), Math.toRadians( 270 ) )
                .lineToSplineHeading( Utils.getHubPosition( -22.5, 270, 7, true ))
                .addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
                } )
                .waitSeconds( 0.8 )

                // move to grab block 3
                .setTangent( Math.toRadians( 90 ) )
                //.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                .lineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 180 ) ))
                .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
                } )
                //.lineToConstantHeading( new Vector2d( 52, wallPos ) ) // 50
                //.lineToConstantHeading( new Vector2d( 12, wallPos ) )
                .forward(40)
                .back(40)
                .addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
                } )

                // move to dump block 3 in the top layer
                .addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
                } )
                //.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ), Math.toRadians( 270 ) )
                .lineToSplineHeading( Utils.getHubPosition( -22.5, 270, 7, true ))
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
                //.turn( Math.toRadians( 110 ) )
                /*// move to barrier to park
                .setTangent( Math.toRadians( 90 ) )
                .splineToSplineHeading( new Pose2d( 11.5, 44, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
                .lineToConstantHeading( new Vector2d( 62, 44 ) )*/
                .build();

        robot.followTrajectorySequence( mainTrajectory );


        stopThreads();

    }
}
