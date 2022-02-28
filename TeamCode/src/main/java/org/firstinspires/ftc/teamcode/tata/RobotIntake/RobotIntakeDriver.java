package org.firstinspires.ftc.teamcode.tata.RobotIntake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class RobotIntakeDriver implements Runnable{
    private RobotIntakeHW hw = new RobotIntakeHW();

    private double delta_x   = 0.0;
    private boolean is_done  = true;
    private boolean intake_on  = false;
    private double motor_power = 1.0;
    private double intake_power = motor_power;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotIntakeDriver(HardwareMap ahwMap, int threadSleepDelay){
        hw.init(ahwMap);
        sleepTime = threadSleepDelay;
    }

    private void robotIntakeAction(){
        if (is_done == false) {
            is_done = true;
            if (intake_on) {
                hw.setPower(intake_power);
            } else {
                hw.setPower(0.0);
            }
        }
    }

    public RobotIntakeParams getRobotIntakeParams(){
        RobotIntakeParams param = new RobotIntakeParams();
        param.powerI0 = hw.getPower();
        return param;
    }

    public void toggleIntake(boolean reverse){
        if (is_done == true) {
            is_done = false;
            intake_on = !intake_on;
            if (reverse == true){
                intake_power = -1*motor_power;
            }else {
                intake_power = motor_power;
            }
        }
    }
    public void intakeSet(boolean isOn, boolean isCollecting){
        //manav function
       if(!isOn){
           hw.stopIntake();
       }else{
           if(isCollecting){
               hw.startIntake(-1.0);
           }else{
               hw.startIntake(1.0);
           }
       }
    }
    public void checkGamePad(Gamepad gp) {
        if (gp.left_trigger > 0.5) {
            hw.setPower(intake_power);
        } else if (gp.right_trigger > 0.5) {
            hw.setPower(-1*intake_power);
        } else {
            hw.setPower(0.0);
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
