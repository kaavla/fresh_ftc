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
		return drive.trajectorySequenceBuilder( new Pose2d( -42.25, -63.5, Math.toRadians( 90 ) ) )
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Capture Pos
				} )
				//Pos 1
				//.lineToSplineHeading(new Pose2d(-44, -47.5, Math.toRadians(90)))
				//Pos 2
				//.lineToSplineHeading(new Pose2d(-36, -47.5, Math.toRadians(90)))
				//Pos 3
				.lineToSplineHeading(new Pose2d(-27, -47.5, Math.toRadians(90)))
				.addTemporalMarker( ( ) -> {
					//Extend Robot Slides to right Height
				} )

				.forward(3)
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Save Pos
				} )
				.lineToSplineHeading(new Pose2d(-44, -24, Math.toRadians(180)))
				.back(10)
				.addTemporalMarker( ( ) -> {
					//Drop Element
				} )
				.waitSeconds( 2 )
				.addTemporalMarker( ( ) -> {
					//Draw Sides in
				} )

				.lineToLinearHeading(new Pose2d(-60, -45, Math.toRadians(270)))
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 1)
				.strafeRight(5)
				.waitSeconds( 1)
				.forward(10)
				.addTemporalMarker( ( ) -> {
                  //start Carousel
				} )
				.waitSeconds( 4)
				.lineToLinearHeading(new Pose2d(-65, -40, Math.toRadians(270)))


				.build( );




	}



}
