package org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


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

    public RobotCaroselDriver(HardwareMap ahwMap, int threadSleepDelay){
        hw.init(ahwMap);
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

    public void toggleCarousel(boolean reverse){
        if (is_done == true) {
            is_done = false;
            carousel_on = !carousel_on;
            if (reverse == true){
                carousel_power = -1*motor_power;
            }else {
                carousel_power = motor_power;
            }
        }
    }

    public void checkGamePad(Gamepad gp) {
        if (gp.b) {
            toggleCarousel(false);
        } else if (gp.x) {
            toggleCarousel(false);
        } else if (gp.y) {
            toggleCarousel(false);
        } else if (gp.a) {
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
