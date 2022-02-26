package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

public class MeepMeepPaths {

	static double robotLength = 18;
	static double robotWidth = 11.5;

	static final double tileSize = 23;
	static final double tileConnector = 0.75;

	static final double hubRadius = 9;

	static final double cameraRightIndent = 1.25;

	public static void main( String[] args ) {
		//BlueInAutoDuck rt = new BlueInAutoDuck();
		//BlueInAutoFreight2 rt = new BlueInAutoFreight2();
		RedInAutoFreight2 rt = new RedInAutoFreight2();
		//BlueOutAutoDuck rt = new BlueOutAutoDuck();
		//BOutDuckTest rt = new BOutDuckTest();
		//PositionGrapherTest rt = new PositionGrapherTest();
		//TechnovaKillerAuto rt = new TechnovaKillerAuto();
		//ShankRed1 rt = new ShankRed1();
		//ShankBlue1 rt = new ShankBlue1();
		//ShankBlue2 rt = new ShankBlue2();
		MeepMeep mm = new MeepMeep( 700 )
				.setBackground( MeepMeep.Background.FIELD_FREIGHT_FRENZY )
				.setTheme( new ColorSchemeRedDark() )
//				.set
				.setBackgroundAlpha( 1f )
				.setBotDimensions( robotWidth, robotLength )
				.setConstraints( 40, 40, 3, 4, 14.1 )
				.followTrajectorySequence( drive -> {
					//try {
						return rt.getTrajectorySequence( drive );
					//} catch( InstantiationException | IllegalAccessException | ClassNotFoundException e ) {
					//	e.printStackTrace( );
				//	}
					//return new TrajectorySequence( new ArrayList<>(  ) );
				} )
				.start( );
	}


}