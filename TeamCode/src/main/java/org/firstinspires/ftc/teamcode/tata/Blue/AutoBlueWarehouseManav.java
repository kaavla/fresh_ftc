package org.firstinspires.ftc.teamcode.tata.Blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.PoseStorage;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous(name = "Blue - Auto - Warehouse - Manav", group = "BLUE")
public class AutoBlueWarehouseManav extends tataAutonomousBase {

    public Pose2d startPose = new Pose2d( 6.75, 60.5, Math.toRadians( 90 ) );


    //Optimal pos for dropping in Levels 1, 2 or 3
    public Pose2d dropPose[] = {new Pose2d(),                            //Not Used
            new Pose2d(    -3.5 , 44.07, Math.toRadians( 67.5 )),  //level 1
            new Pose2d( -2.3, 46.8, Math.toRadians( 67.5 )),  //level 2
            new Pose2d( -1.9,46.8, Math.toRadians( 67.5 ))}; //level 3

    private double wallPos = 63;

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
        //stopMainCamera();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        TrajectorySequence moveToDropGE = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    moveSlideToPos(barCodeLoc, SlideDirection.OUT);
                })
                .lineToSplineHeading(dropPose[barCodeLoc])
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    slideDriver.dropGameElement();
                })
                .waitSeconds(0.2)

                .addTemporalMarker(() -> {
                    moveSlideToPos(barCodeLoc, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .setTangent( Math.toRadians( 75) )
                .splineToSplineHeading( new Pose2d( 14, wallPos , Math.toRadians( 0 ) ), Math.toRadians( 0))
                .lineToConstantHeading( new Vector2d( 42, 63 ) )
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(true, true);
                })

                .build();
        robot.followTrajectorySequence(moveToDropGE);
        PoseStorage.currentPose = robot.getPoseEstimate();

        //Correct Robot Orientation
        //imuParams = imuDriver.getRobotImuParams();
        //robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - 270));

        dsParams = sensorDriver.getRobotSensorParams();

        RobotLog.ii("SHANK", "Duck Side Red - RS %.2f, F %.2f", dsParams.x_RS, dsParams.x_LF);



        //Check if duck has been collected
        TrajectorySequence collectDuck = getTrajectorySequenceBuilder()
                .forward(8)
                .build();

        ElapsedTime stopTimer = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested() && (stopTimer.seconds() < 2))
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
                .back(10)
                .strafeLeft(3)
                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.OUT);
                })
                .setTangent(Math.toRadians(180))
                .lineToConstantHeading( new Vector2d( 14, 63 ) )
                .splineToLinearHeading(dropPose[3], Math.toRadians(180+67.5))

                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    slideDriver.dropGameElement();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .setTangent( Math.toRadians( 75) )
                .splineToSplineHeading( new Pose2d( 14, wallPos , Math.toRadians( 0 ) ), Math.toRadians( 0))
                .lineToConstantHeading( new Vector2d( 42, 63 ) )
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(true, true);
                })

                .build();
        robot.followTrajectorySequence(moveToDropDuck);
        TrajectorySequence collectDuck1 = getTrajectorySequenceBuilder()
                .forward(8)
                .build();

        ElapsedTime stopTimer1 = new ElapsedTime();
        while(opModeIsActive() && !isStopRequested() && (stopTimer1.seconds() < 2))
        {
            if (inTakeDriver.isElementCollected()) {
                //Game Elemented is collected
                break;
            }

            robot.followTrajectorySequence(collectDuck);
            PoseStorage.currentPose = robot.getPoseEstimate();
        }
        TrajectorySequence moveToDropDuck1 = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    inTakeDriver.intakeSet(false, true);
                })
                .back(10)
                .strafeLeft(3)
                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.OUT);
                })
                .setTangent(Math.toRadians(180))
                .lineToConstantHeading( new Vector2d( 14, 63 ) )
                .splineToLinearHeading(dropPose[3], Math.toRadians(180+67.5))

                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    slideDriver.dropGameElement();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    moveSlideToPos(3, SlideDirection.IN);
                    //sideArmDriver.activateSideArms(RobotSideArmDriver.RobotSideArmPreSetPos.DOWN);
                })
                .build();
                robot.followTrajectorySequence(moveToDropDuck1);
        sleep(200);


        stopThreads();

    }
}
