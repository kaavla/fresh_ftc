package org.firstinspires.ftc.teamcode.tata.Common;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC.RobotCaroselDriver;
import org.firstinspires.ftc.teamcode.tata.RobotFrontServo.RobotFrontServoDriver;
import org.firstinspires.ftc.teamcode.tata.RobotImu.RobotImuDriver;
import org.firstinspires.ftc.teamcode.tata.RobotImu.RobotImuParams;
import org.firstinspires.ftc.teamcode.tata.RobotIntake.RobotIntakeDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.tata.RobotSlide.RobotSlideDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class tataAutonomousBase extends LinearOpMode {

    public enum SideColor {
        Red, Blue
    }

    public enum StartPos {
        Warehouse, //White Colored Box
        Storage    //Red or Blue Colored box
    }

    public tataMecanumDrive robot;
    public RobotSensorDriver sensorDriver;
    public RobotSensorParams params;

    public RobotIntakeDriver inTakeDriver;
    public RobotSlideDriver slideDriver;
    public RobotArmDriver armDriver;
    public RobotCaroselDriver crDriver;
    public RobotFrontServoDriver frDriver;

    public RobotImuDriver imuDriver;
    public RobotImuParams imuParams;

    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Empty Function
    }

    public void init(HardwareMap hwMap, Pose2d startPose) {
        robot = new tataMecanumDrive(hardwareMap);

        sensorDriver = new RobotSensorDriver(hwMap, 100);
        Thread sensorDriverThread = new Thread(sensorDriver);
        sensorDriverThread.start();

        inTakeDriver = new RobotIntakeDriver(hwMap, 2000);
        Thread intakeDriverThread = new Thread(inTakeDriver);
        intakeDriverThread.start();

        slideDriver = new RobotSlideDriver(hwMap, 50, true);
        Thread slideDriverThread = new Thread(slideDriver);
        slideDriverThread.start();

        armDriver = new RobotArmDriver(hwMap, 50);
        Thread armDriverThread = new Thread(armDriver);
        armDriverThread.start();

        crDriver = new RobotCaroselDriver(hwMap, 200);
        Thread crDriverThread = new Thread(crDriver);
        crDriverThread.start();

        frDriver = new RobotFrontServoDriver(hwMap, 2000);
        Thread frDriverThread = new Thread(frDriver);
        frDriverThread.start();

        imuDriver = new RobotImuDriver(hardwareMap, 300, Math.toDegrees(startPose.getHeading()));
        Thread imuDriverThread = new Thread(imuDriver);
        imuDriverThread.start();

    }

    public TrajectorySequenceBuilder getTrajectorySequenceBuilder() {
        return robot.trajectorySequenceBuilder(robot.getPoseEstimate());
    }

    public double getSlideHeightByLvlInInch(int lvl) {
        switch (lvl) {
            case 1:
                return 8.0;
            case 2:
                return 10.0;
            case 3:
                return 21.0;
        }
        return 0.0;
    }

    public Pose2d getTeamMarkerCoord(SideColor sc, StartPos sp, int lvl) {
        Pose2d pose = new Pose2d(0, 0, 0);
        if (sc == SideColor.Red) {
            if (sp == StartPos.Storage) {
                switch (lvl) {
                    case 1: {
                        pose = new Pose2d(-45, -47.5, Math.toRadians(90));
                        break;
                    }
                    case 2: {
                        pose = new Pose2d(-36, -47.5, Math.toRadians(90));
                        break;
                    }
                    case 3: {
                        pose = new Pose2d(-27, -47.5, Math.toRadians(90));
                        break;
                    }
                }
            } else {
                //sp == Warehouse
                switch (lvl) {
                    case 1: {
                        pose = new Pose2d(-44.5, -47.5, Math.toRadians(90));
                        break;
                    }
                    case 2: {
                        pose = new Pose2d(-36, -47.5, Math.toRadians(90));
                        break;
                    }
                    case 3: {
                        pose = new Pose2d(-27, -47.5, Math.toRadians(90));
                        break;
                    }
                }

            }
        } else {
            //Blue side
            if (sp == StartPos.Storage) {
                switch (lvl) {
                    case 1: {
                        pose = new Pose2d(-27, 47.5, Math.toRadians(270));
                        break;
                    }
                    case 2: {
                        pose = new Pose2d(-36, 47.5, Math.toRadians(270));
                        break;
                    }
                    case 3: {
                        pose = new Pose2d(-45, 47.5, Math.toRadians(270));
                        break;
                    }
                }
            } else {
                //sp == Warehouse
                switch (lvl) {
                    case 1: {
                        pose = new Pose2d(4, 36, Math.toRadians(90));
                        break;
                    }
                    case 2: {
                        pose = new Pose2d(8, 36, Math.toRadians(90));
                        break;
                    }
                    case 3: {
                        pose = new Pose2d(12, 36, Math.toRadians(90));
                        break;
                    }
                }

            }

        }
        return pose;
    }


    public void stopThreads() {
        sensorDriver.stop();
        inTakeDriver.stop();
        slideDriver.stop();
        armDriver.stop();
        crDriver.stop();
        frDriver.stop();
        imuDriver.stop();

    }
}


