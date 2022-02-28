package org.firstinspires.ftc.teamcode.tata.Manual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.Common.tataMecanumDrive;
import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name = "Manual Mode", group = "Linear Opmode")

public class ManualDrive extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-42.25, -63.5, Math.toRadians(90));

    public void autoRunToPos() {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        Pose2d pose = new Pose2d(10, 10, Math.toRadians(90));

        TrajectorySequence dropElement = getTrajectorySequenceBuilder()
                .addTemporalMarker(() -> {
                    moveSlideToPos(0, SlideDirection.OUT);
                })
                .lineToSplineHeading(pose)
                .addTemporalMarker(() -> {
                    slideDriver.dropGameElement();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    moveSlideToPos(0, SlideDirection.IN);
                })
                //.waitSeconds(1.0)
                .build();
        robot.followTrajectorySequence(dropElement);

    }

        @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) {
            stopThreads();
            return;
        }


        while (!isStopRequested()) {
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            if (gamepad1.x) {
                autoRunToPos();
            }
            /*
            inTakeDriver.checkGamePad(gamepad1);
            slideDriver.checkGamePad(gamepad2);
            slideDriver.checkGamePadX(gamepad1); //only to rotate the arm

            armDriver.checkGamePad(gamepad2);
            crDriver.checkGamePad(gamepad2);
            frDriver.checkGamePad(gamepad1);
*/
            idle();
            robot.update();

        }
    }


       // stopThreads();

    }
//}
