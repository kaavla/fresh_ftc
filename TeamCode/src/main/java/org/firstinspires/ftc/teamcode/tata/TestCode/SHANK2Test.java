package org.firstinspires.ftc.teamcode.tata.TestCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;

@Autonomous(name = "RED - SHANK2 - AllianceStorage", group = "RED")
@Disabled
public class SHANK2Test extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-42.25, -63.5, Math.toRadians(90));

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


        traj1 = robot.trajectoryBuilder(startPose)
                //.lineToLinearHeading(new Pose2d(-12, -43, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-24, -24, Math.toRadians(180)))
                .build();
        robot.followTrajectory(traj1);
        traj_last = traj1;
        //Check correct
        sleep(1000);
        //imuParams = imuDriver.getRobotImuParams();
        //apply correction
        //robot.turn(-1*Math.toRadians(imuParams.correctedHeading - 180));

        //telemetry.addData("Rel:", "%2f deg ", imuParams.correctedHeading);
        //telemetry.addData("diff:", "%2f deg ", Math.abs(imuParams.correctedHeading - 180));
        telemetry.update();

        sleep(3000);

        //go to carousel
        /*
        traj5 = robot.trajectoryBuilder(traj_last.end().plus(new Pose2d(0, 0, -1*Math.toRadians(imuParams.correctedHeading - 180))))
                .lineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(270)))
                .build();
        robot.followTrajectory(traj5);

         */
        traj_last = traj1;

        //Check correct
        sleep(1000);
        //imuParams = imuDriver.getRobotImuParams();
        //apply correction
        //robot.turn(-1*Math.toRadians(imuParams.correctedHeading - 270));
        sleep(100);

        RobotSensorParams dsParams = sensorDriver.getRobotSensorParams();

        //telemetry.addData("Rel:", "%2f deg ", imuParams.correctedHeading);
        //telemetry.addData("diff:", "%2f deg ", Math.abs(imuParams.correctedHeading - 270));
        telemetry.addData("x_LF:", "%2f:%2f deg ", dsParams.x_LF, dsParams.x_LF1);
        telemetry.update();

        sleep(6000);
        dsParams = sensorDriver.getRobotSensorParams();
        //imuParams = imuDriver.getRobotImuParams();
        //telemetry.addData("Rel:", "%2f deg ", imuParams.correctedHeading);
        //telemetry.addData("diff:", "%2f deg ", Math.abs(imuParams.correctedHeading - 270));
        telemetry.addData("x_LF:", "%2f:%2f deg ", dsParams.x_LF, dsParams.x_LF1);
        telemetry.update();
        sleep(6000);

        stopThreads();

    }
}
