package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class ShankManualBlueSharedHub implements MeepMeepPath{

	double wallPos = 63;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 63, 38, Math.toRadians( 90 ) ) )
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Capture Pos
				} )
				.back(18)
				.waitSeconds(1)
				//.lineToSplineHeading(new Pose2d(-9,42 , Math.toRadians(82)))
				.setTangent( Math.toRadians( 270-45) )
				.splineToLinearHeading( new Pose2d(59.9,13.5 , Math.toRadians(45)), Math.toRadians(270-45 ) )


				.addTemporalMarker( ( ) -> {
					//Extend Robot Slides to right Height
				} )
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Capture Pos
				} )
				.setTangent( Math.toRadians( 45) )
				.splineToLinearHeading( new Pose2d(63,24 , Math.toRadians(90)), Math.toRadians(90 ) )
				.forward(24)


				.build( );
	}



}
