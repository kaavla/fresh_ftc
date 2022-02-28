package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class ShankManualBlue implements MeepMeepPath{

	double wallPos = 63;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 63, 23.75, Math.toRadians( 90 ) ) )
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Capture Pos
				} )
				.back(6)
				.lineToSplineHeading(new Pose2d(60, 13,  Math.toRadians( 45 )))
				.setTangent( Math.toRadians( 270 ) )
				//.splineToLinearHeading( MeepMeepPath.getHubPositionX( -45, 90, 8, true ), Math.toRadians( 0  ) )
				//.splineToLinearHeading( new Pose2d( 63.9, 9.5,  Math.toRadians(30 ) ), Math.toRadians( 200 ) )

				.addTemporalMarker( ( ) -> {
					//Extend Robot Slides to right Height
				} )
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Capture Pos
				} )
				.lineToSplineHeading( new Pose2d( 63, 23.75, Math.toRadians( 90 ) ))
				.setTangent( Math.toRadians( 270 ) )
				.forward(10)
				//.splineToLinearHeading( MeepMeepPath.getHubPositionX( -45, 90, 8, true ), Math.toRadians( 0  ) )
				//.splineToLinearHeading( new Pose2d( 63.9, 9.5,  Math.toRadians(30 ) ), Math.toRadians( 200 ) )

				.addTemporalMarker( ( ) -> {
					//Extend Robot Slides to right Height
				} )



				.build( );
	}



}
