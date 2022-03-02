package org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;


public class RobotCaroselDriver implements Runnable{
    private RobotCarouselHW hw = new RobotCarouselHW();

    private double delta_x   = 0.0;
    private boolean is_done  = true;
    private boolean carousel_on  = false;
    private double motor_power = 0.95;
    private double carousel_power = motor_power;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotCaroselDriver(HardwareMap ahwMap, int threadSleepDelay, tataAutonomousBase.SideColor sc){
        hw.init(ahwMap, sc);
        sleepTime = threadSleepDelay;
    }

    private void robotIntakeAction(){
        if (is_done == false) {
            is_done = true;
            if (carousel_on) {
                hw.setPower(carousel_power);
            } else {
                hw.setPower(0.0);
            }
        }
    }

    public RobotCarouselParams getRobotIntakeParams(){
        RobotCarouselParams param = new RobotCarouselParams();
        param.power = hw.getPower();
        return param;
    }

    public void toggleCarousel(boolean switchOn){
        if (is_done == true) {
            is_done = false;
            carousel_on = switchOn;
        }
    }

    public void checkGamePad(Gamepad gp) {
        if (gp.left_trigger > 0.5) {
            toggleCarousel(true);
        } else if (gp.right_trigger > 0.5) {
            toggleCarousel(false);
        }

    }


    public void stop(){ isRunning = false; }

    @Override
    public void run() {
        while(isRunning) {
            robotIntakeAction();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
