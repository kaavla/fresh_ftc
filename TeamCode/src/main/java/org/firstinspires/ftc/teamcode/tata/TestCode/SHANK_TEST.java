package org.firstinspires.ftc.teamcode.tata.TestCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.tata.RobotSlide.RobotSlideDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "SHANK test", group = "RED")
public class SHANK_TEST extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-42.25, -63.5, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose);
        robot.setPoseEstimate(startPose);

        //int barCodeLoc = 1;
        int barCodeLoc = sensorDriver.getBarCodeRED();
        RobotSensorParams dsParams = new RobotSensorParams();


        while (!isStopRequested() && !isStarted()) {
            barCodeLoc = sensorDriver.getBarCodeRED();
            telemetry.addData("Waiting to Start. Element position", barCodeLoc);
            telemetry.update();
        }

        waitForStart();
        telemetry.addData("Started. Element position", barCodeLoc);
        telemetry.update();
        barCodeLoc = 3;

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        Pose2d pose = getTeamMarkerCoord(SideColor.Red, StartPos.Storage, barCodeLoc);
        double slideLen = getSlideHeightByLvlInInch(barCodeLoc);
        int lvl = barCodeLoc;

        //slideDriver.moveRobotSlideBy(5,0);
        //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.OUT);
        moveSlideToPos(lvl, SlideDirection.OUT);
        sleep(4000);
        slideDriver.dropGameElement();
        sleep(1000);
        //slideDriver.moveRobotSlideBy(-5,0);
        //slideDriver.moveSlideToDropPos(lvl, RobotSlideDriver.SlideDirection.IN);
        sleep(4000);

        stopThreads();

    }
}
