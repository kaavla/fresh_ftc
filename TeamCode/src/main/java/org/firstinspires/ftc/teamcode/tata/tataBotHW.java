package org.firstinspires.ftc.teamcode.tata;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC.RobotCaroselDriver;
import org.firstinspires.ftc.teamcode.tata.RobotDrivetrain.RobotDriveTrainDriver;
import org.firstinspires.ftc.teamcode.tata.RobotDrivetrain.RobotDrivetrainHW;
import org.firstinspires.ftc.teamcode.tata.RobotFrontServo.RobotFrontServoDriver;
import org.firstinspires.ftc.teamcode.tata.RobotImu.RobotImuDriver;
import org.firstinspires.ftc.teamcode.tata.RobotImu.RobotImuParams;
import org.firstinspires.ftc.teamcode.tata.RobotIntake.RobotIntakeDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.tata.RobotSlide.RobotSlideDriver;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class tataBotHW {
    public RobotDriveTrainDriver dtDriver;
    public RobotSensorDriver sensorDriver;
    public RobotIntakeDriver inTakeDriver;
    public RobotSlideDriver  slideDriver;
    //public RobotArmDriver    armDriver;
    public RobotCaroselDriver crDriver;
    public RobotFrontServoDriver frDriver;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        RobotLog.ii("CAL", "Enter - init");

        dtDriver = new RobotDriveTrainDriver(hwMap, 50);
        Thread dtDriverThread = new Thread(dtDriver);
        dtDriverThread.start();

        sensorDriver = new RobotSensorDriver(hwMap, 100);
        Thread sensorDriverThread = new Thread(sensorDriver);
        sensorDriverThread.start();

        inTakeDriver = new RobotIntakeDriver(hwMap, 2000, telemetry);
        Thread intakeDriverThread = new Thread(inTakeDriver);
        intakeDriverThread.start();

        slideDriver  = new RobotSlideDriver(hwMap, 50, false);
        Thread slideDriverThread = new Thread(slideDriver);
        slideDriverThread.start();

        //armDriver    = new RobotArmDriver(hwMap, 50);
        //Thread armDriverThread = new Thread(armDriver);
        //armDriverThread.start();

        crDriver     = new RobotCaroselDriver(hwMap, 200, tataAutonomousBase.SideColor.Blue);
        Thread crDriverThread = new Thread(crDriver);
        crDriverThread.start();

        frDriver     = new RobotFrontServoDriver(hwMap, 2000);
        Thread frDriverThread = new Thread(frDriver);
        frDriverThread.start();

    }
    public void stopThreads() {
        dtDriver.stop();
        sensorDriver.stop();
        inTakeDriver.stop();
        slideDriver.stop();
        //armDriver.stop();
        crDriver.stop();
        frDriver.stop();
    }



    }
