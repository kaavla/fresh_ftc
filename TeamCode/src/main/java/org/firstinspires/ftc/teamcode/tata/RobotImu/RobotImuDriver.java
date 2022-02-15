package org.firstinspires.ftc.teamcode.tata.RobotImu;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC.RobotCarouselHW;
import org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC.RobotCarouselParams;


public class RobotImuDriver implements Runnable{
    private RobotImuHW hw = new RobotImuHW();

    private boolean is_done  = true;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotImuDriver(HardwareMap ahwMap, int threadSleepDelay, double refAngle){
        hw.init(ahwMap, refAngle);
        sleepTime = threadSleepDelay;
    }

    public RobotImuParams getRobotImuParams(){
        RobotImuParams param = new RobotImuParams();
        param.rawHeading = hw.getRawHeading();
        param.correctedHeading = hw.getRelativeHeading();
        param.correction= hw.getCorrection();
        return param;
    }
    public void robotImuTriggerRead(){
        hw.readImu();
    }

    public void stop(){ isRunning = false; }

    @Override
    public void run() {
        while(isRunning) {
            robotImuTriggerRead();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
