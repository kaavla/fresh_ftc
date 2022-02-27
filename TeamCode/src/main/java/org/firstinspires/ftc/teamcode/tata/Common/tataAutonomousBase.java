package org.firstinspires.ftc.teamcode.tata.Common;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC.RobotCaroselDriver;
import org.firstinspires.ftc.teamcode.tata.RobotFrontServo.RobotFrontServoDriver;
import org.firstinspires.ftc.teamcode.tata.RobotImu.RobotImuDriver;
import org.firstinspires.ftc.teamcode.tata.RobotImu.RobotImuParams;
import org.firstinspires.ftc.teamcode.tata.RobotIntake.RobotIntakeDriver;
import org.firstinspires.ftc.teamcode.tata.RobotLinearActuators.RobotLinearActuatorDriver;
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

    public enum SlideDirection {
        IN,
        OUT
    }

    public tataMecanumDrive robot;
    public RobotSensorDriver sensorDriver;
    public RobotSensorParams params;

    public RobotIntakeDriver inTakeDriver;
    public RobotSlideDriver slideDriver;
    public RobotArmDriver armDriver;
    public RobotCaroselDriver crDriver;
    public RobotFrontServoDriver frDriver;

    public RobotLinearActuatorDriver driver0;
    public RobotLinearActuatorDriver driver1;

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

        driver0 = new RobotLinearActuatorDriver(hardwareMap, 1000, 0);
        Thread driverThread0 = new Thread(driver0);
        driverThread0.start();

        driver1 = new RobotLinearActuatorDriver(hardwareMap, 1000, 1);
        Thread driverThread1 = new Thread(driver1);
        driverThread1.start();


    }

    public TrajectorySequenceBuilder getTrajectorySequenceBuilder() {
        return robot.trajectorySequenceBuilder(robot.getPoseEstimate());
    }

    public void moveSlideToPos(int lvl, SlideDirection slideDirection) {
        //0th element should be ignored as levels are 1, 2, 3
        double slideDistanceInIncPerLevel[] = {0, 5.0, 9.5, 17.0};
        double slideInclinePerLevel[]       = {0, 0.0, 0.1,  0.2};

        if (slideDirection == SlideDirection.OUT) {
            slideDriver.moveRobotSlideBy(slideDistanceInIncPerLevel[lvl], 0);
            driver0.pullLinearActuatorBy(-1 * slideInclinePerLevel[lvl]); //Pull up
            driver1.pullLinearActuatorBy(-1 * slideInclinePerLevel[lvl]);
        } else {
            //IN
            driver0.pullLinearActuatorBy(slideInclinePerLevel[lvl]); //pull down
            driver1.pullLinearActuatorBy(slideInclinePerLevel[lvl]); //pull down
            slideDriver.moveRobotSlideBy(-1 * slideDistanceInIncPerLevel[lvl], 0);
        }
    }

/*
    target   Imu   direction to move
        0         10   CW 10    (target - imu = -10)
        0         350  CCW 10   (target - imu = 0 - (-10) = 10)

       270       280  CW 10    (target - imu = -10)
       270       260  CCW 10   (target - imu = 10)

       So whatever is returned by this functiom.. just turn in same direction
*/
    public double correctOrientationUsingImu(double targetHeading) {

        imuParams = imuDriver.getRobotImuParams();
        double correctedHeading =imuParams.correctedHeading;

        RobotLog.ii("SHANK", "targetHeading %2f, corrected heading %2f", targetHeading, correctedHeading);

        //Correction of 0 target heading because of 360 wrap around
        if (targetHeading == 0) {
            if (correctedHeading > 180) {
                correctedHeading = correctedHeading - 360;
            }
        }
        RobotLog.ii("SHANK", "(targetHeading - corrected heading )%2f", (targetHeading - correctedHeading));

        return (targetHeading - correctedHeading);
    }
    public double getDistanceFromWall(SideColor sc) {
        RobotSensorParams rsp = sensorDriver.getRobotSensorParams();
        double T;
        if (sc == SideColor.Blue) {
            T =  rsp.x_LS;
            RobotLog.ii("SHANK", "Blue LS =  %2f", T);

        } else {
            T = rsp.x_RS;
            RobotLog.ii("SHANK", "Red RS =  %2f", T);
        }

        return (T - 1);
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
                        pose = new Pose2d(-44, -52, Math.toRadians(90));//-45,-47.5
                        break;
                    }
                    case 2: {
                        pose = new Pose2d(-36, -51.5, Math.toRadians(90));
                        break;
                    }
                    case 3: {
                        pose = new Pose2d(-29, -52, Math.toRadians(90));
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
        driver0.stop();
        driver1.stop();

    }
}


