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

public class tataAutonomousBase extends LinearOpMode {


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


