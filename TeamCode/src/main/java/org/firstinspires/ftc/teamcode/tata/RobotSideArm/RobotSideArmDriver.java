package org.firstinspires.ftc.teamcode.tata.RobotSideArm;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmHW;
import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmParams;

public class RobotSideArmDriver implements Runnable {
    public enum RobotSideArmPreSetPos {
        DOWN,
        UP,
        INVALID
    }
    private RobotSideArmHW sideArmHW = new RobotSideArmHW();
    private RobotSideArmPreSetPos sideArmPos;

    private boolean is_done = true;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotSideArmDriver(HardwareMap ahwMap, int threadSleepDelay) {
        sideArmHW.init(ahwMap);
        sleepTime = threadSleepDelay;
        sideArmPos = RobotSideArmPreSetPos.UP;
    }

    private void robotArmPositionUpdate() {
        if (is_done == false) {
            is_done = true;
            if (sideArmPos == RobotSideArmPreSetPos.UP) {
                sideArmHW.servoSetPosRaw(0, 0.0);
                try {
                    Thread.sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                sideArmHW.servoSetPosRaw(1, 0.0);
                try {
                    Thread.sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            } else {
                //DOWN
                sideArmHW.servoSetPosRaw(0, 0.7);
                try {
                    Thread.sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                sideArmHW.servoSetPosRaw(1, 0.7);
                try {
                    Thread.sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        }
    }

    public void checkGamePad(Gamepad gp) {
    }


    public void activateSideArms(RobotSideArmPreSetPos pos) {
        if (is_done == true) {
            is_done = false;
            sideArmPos = pos;
        }
    }

    public void stop() {
        isRunning = false;
    }

    @Override
    public void run() {
        while (isRunning) {
            robotArmPositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
