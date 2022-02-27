package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class BlueInAutoFreight2 implements MeepMeepPath{

	double wallPos = 63;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 6, 61, Math.toRadians( 270 ) ) )
				.setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint( 70 ), new MecanumVelocityConstraint( 40, 17 ) ) ) )

				// move to dump initial block in designated layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( height );
				} )
				.setTangent( Math.toRadians( 270 ) )
				//.splineToLinearHeading( MeepMeepPath.getHubPositionX( -45, 90, 8, true ), Math.toRadians( 0  ) )
				.splineToLinearHeading( new Pose2d( -4.3,42.3 , Math.toRadians(67.5) ), Math.toRadians( 300 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 0.8 )

				// move to grab block 1
				.setTangent( Math.toRadians( 100 + 0) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 0) )
				//.lineToLinearHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ) )
				//.addTemporalMarker( ( ) -> {
//				//	robot.intake.setPower( 0.6 );
				//} )
				.lineToLinearHeading(new Pose2d(48, wallPos+1, Math.toRadians(0)))
				.lineToLinearHeading(new Pose2d(18, wallPos+1, Math.toRadians(0)))
				//.lineToConstantHeading( new Vector2d( 48, wallPos ) ) // 48
				//.lineToConstantHeading( new Vector2d( 18, wallPos ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
				} )

				// move to dump block 1 in the top layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.setTangent( Math.toRadians( 200) )
				.splineToLinearHeading( new Pose2d( 2.26,37.9 , Math.toRadians(45) ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 0.8 )

				// move to grab block 2
				.setTangent( Math.toRadians( 90 + 0) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 2 ) ), Math.toRadians( 10) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 48, wallPos ) ) // 48
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
				} )

				// move to dump block 1 in the top layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.setTangent( Math.toRadians( 200) )
				.splineToLinearHeading( new Pose2d( 2.26,37.9 , Math.toRadians(45) ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 0.8 )

				// move to grab block 3
				.setTangent( Math.toRadians( 90 + 0) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 2 ) ), Math.toRadians( 10) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 48, wallPos ) ) // 48
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
				} )

				// move to dump block 1 in the top layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.setTangent( Math.toRadians( 200) )
				.splineToLinearHeading( new Pose2d( 2.26,37.9 , Math.toRadians(45) ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 0.8 )

				// Park
				.setTangent( Math.toRadians( 90 + 0) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 2 ) ), Math.toRadians( 10) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 48, wallPos ) ) // 48







/*
				// move to grab block 2
				.setTangent( Math.toRadians( 180 - 45) )
				.splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 45 ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 50, wallPos ) ) // 53
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
				} )

				// move to dump block 2 in the top layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.setTangent( Math.toRadians( 180) )
				.splineToLinearHeading( MeepMeepPath.getHubPositionX( -45, 90, 8, true ), Math.toRadians( 0  ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 0.8 )

				// move to grab block 3
				.setTangent( Math.toRadians( 180 - 45) )
				.splineToSplineHeading( new Pose2d( 12, wallPos, Math.toRadians( 0 ) ), Math.toRadians( 45 ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 52, wallPos ) ) // 50
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
				} )

				// move to dump block 3 in the top layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )

				// turn towards the
				.turn( Math.toRadians( 110 ) )
		*/
				.build( );




	}



}
