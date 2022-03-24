package org.firstinspires.ftc.teamcode.tata.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.PoseStorage;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorHW;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.tata.RobotSideArm.RobotSideArmDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous(name = "RED - Final Alliance Path", group = "RED")
public class FinalRedAlliance extends tataAutonomousBase {

    public Pose2d startPose = new Pose2d( -40.75, -63.5, Math.toRadians( 270 ));

    //public Pose2d dropPose1 = new Pose2d( -30.9, -34.75, Math.toRadians( 180+30 ));
    //public Pose2d dropPose2 = new Pose2d( -33.5, -36.25, Math.toRadians( 180+30 ));
    //public Pose2d dropPose3 = new Pose2d( -37.85,-38.75, Math.toRadians( 180+30 ));

    //Optimal pos for dropping in Levels 1, 2 or 3
    public Pose2d dropPose[] = {new Pose2d(),                            //Not Used
            new Pose2d( -30.9, -34.75, Math.toRadians( 180+30 )),  //level 1
            new Pose2d( -33.5, -36.25, Math.toRadians( 180+30 )),  //level 2
            new Pose2d( -37.85,-38.75, Math.toRadians( 180+30 ))}; //level 3


    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose, opModeCalled.AUTO);
        robot.setPoseEstimate(startPose);

        while (!isStopRequested() && !isStarted()) {
            barCodeLoc = getMarkerPos();
            telemetry.addData("Waiting to Start. Element position", barCodeLoc);
            telemetry.update();
        }

        waitForStart();

        telemetry.addData("Started. Element position", barCodeLoc);
        telemetry.update();

        //Dont need Main camera Anymore
        stopMainCamera();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                   // moveSlideToPos(barCodeLoc, SlideDirection.OUT);
                })
                .lineToSplineHeading(dropPose[barCodeLoc])
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                   // slideDriver.dropGameElement();
                })
                .waitSeconds(0.1)

                .addTemporalMarker(() -> {
                   // moveSlideToPos(barCodeLoc, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(270)))
                .build();
        robot.followTrajectorySequence(moveToDropGE);
        PoseStorage.currentPose = robot.getPoseEstimate();

        //Correct Robot Orientation
        imuParams = imuDriver.getRobotImuParams();
        robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - 270));

        dsParams = sensorDriver.getRobotSensorParams();

        RobotLog.ii("SHANK", "Duck Side Red - RS %.2f, F %.2f", dsParams.x_RS, dsParams.x_LF);


        TrajectorySequence moveToStartCarousel = getTrajectorySequenceBuilder()
                .setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL ), new MecanumVelocityConstraint( 15, DriveConstants.TRACK_WIDTH ) ) ) )
                .strafeRight(Math.min(dsParams.x_RS - 7, 6))
                .waitSeconds(0.2)
                .forward(Math.min(dsParams.x_LF - 8, 12))  //Carousel if of radius 7.5 inch //was using LF
                .addTemporalMarker(() -> {
                    //start Carosel motor
                    crDriver.toggleCarousel(true);
                })
                .waitSeconds(4)
                .addTemporalMarker(() -> {
                    //stop Carosel motor
                    crDriver.toggleCarousel(false);
                })
                .strafeLeft(8)
                .strafeRight(4)
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(true, true);
                })
                .build();

        robot.followTrajectorySequence(moveToStartCarousel);
        PoseStorage.currentPose = robot.getPoseEstimate();

        //Check if duck has been collected
        TrajectorySequence collectDuck = getTrajectorySequenceBuilder()
                .strafeLeft(1)
                .build();

        ElapsedTime stopTimer = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested() && (stopTimer.seconds() < 5))
        {
            if (inTakeDriver.isElementCollected()) {
                //Game Elemented is collected
                break;
            }

            robot.followTrajectorySequence(collectDuck);
            PoseStorage.currentPose = robot.getPoseEstimate();
        }

        //Drop Duck
        TrajectorySequence moveToDropDuck = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(false, true);
                })
                .addTemporalMarker(() -> {
                 //   moveSlideToPos(1, SlideDirection.OUT);
                })
                .lineToSplineHeading(dropPose[1])
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                 //   slideDriver.dropGameElement();
                })
                .waitSeconds(0.1)

                .addTemporalMarker(() -> {
                  //  moveSlideToPos(barCodeLoc, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .lineToLinearHeading(new Pose2d(-62, -20, Math.toRadians(270)))
                .build();
        robot.followTrajectorySequence(moveToDropDuck);

        dsParams = sensorDriver.getRobotSensorParams();
        RobotLog.ii("SHANK", "Park - RS %.2f, F %.2f", dsParams.x_RS, dsParams.x_LF);

        TrajectorySequence movetoPark = getTrajectorySequenceBuilder()
                .forward(1)
                .build();

        ElapsedTime elapsedTime = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested() && (stopTimer.seconds() < 2)) {
            if (dsParams.c_LS == RobotSensorHW.DetectedColors.RED) {
                break;
            }
            robot.followTrajectorySequence(movetoPark);
        }

        TrajectorySequence Park = getTrajectorySequenceBuilder()
                .forward(15)
                .build();
        robot.followTrajectorySequence(Park);

        stopThreads();

    }
}
