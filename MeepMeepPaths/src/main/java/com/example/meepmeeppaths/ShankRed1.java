package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class ShankRed1 implements MeepMeepPath{

	double wallPos = 63;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( -40.75, -63.5, Math.toRadians( 270 ) ) )
				.waitSeconds(3)
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Capture Pos
				} )
				//Pos 1
				//.lineToSplineHeading(new Pose2d(-44, -47.5, Math.toRadians(90)))
				//Pos 2
				//.lineToSplineHeading(new Pose2d(-36, -47.5, Math.toRadians(90)))
				//Pos 3
				//.setTangent( Math.toRadians( 67.5 ) )
				//.splineToSplineHeading( new Pose2d( -21, -46 , Math.toRadians( 180 + 67.5 ) ), Math.toRadians( 67.5))
				.lineToSplineHeading(new Pose2d( -32.6, -35.75 , Math.toRadians( 180 + 30 )))

				.addTemporalMarker( ( ) -> {
					//Extend Robot Slides to right Height
				} )

				.addTemporalMarker( ( ) -> {
					//Draw Sides in
				} )

				.lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(270)))
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 1)
				.strafeRight(5)
				//.waitSeconds( 1)
				//.forward(10)
				.addTemporalMarker( ( ) -> {
                  //start Carousel
				} )
				.lineToSplineHeading(new Pose2d( -32.6, -35.75 , Math.toRadians( 180 + 30 )))


				.lineToLinearHeading(new Pose2d(-65, -35, Math.toRadians(270)))


				.build( );




	}



}
