package org.firstinspires.ftc.teamcode.tata.RobotSlide;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotSlideDriver implements Runnable {
    public enum SlideDirection {
        IN,
        OUT
    }

    public RobotSlideHW slideHW = new RobotSlideHW();

    private double delta_x = 0.0;
    private double delta_i = 0.0;
    private boolean is_done = true;
    private double tiltSlideBoxPos = -1.0;
    private double rotateSlideBoxArmPos = -1.0;
    //Thread run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotSlideDriver(HardwareMap ahwMap, int threadSleepDelay, boolean enableEncoders) {
        slideHW.init(ahwMap);
        sleepTime = threadSleepDelay;

        if (enableEncoders == true) {
            slideHW.initSlideEncoders();
        }
    }

    public void dropGameElement() {
        double tilt_pos_delta = 0.15;
        double tilt_arm_pos_delta = 0.3;
        //Tilt box
        slideHW.setSlideServoCurrPos(0, -1 * tilt_pos_delta);

        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //rotate arm

        slideHW.setSlideServoCurrPos(1, -1 * tilt_arm_pos_delta);
        //slideHW.rotateBoxArm(0.8);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //Reset Box
        slideHW.setSlideServoCurrPos(0, 1 * tilt_pos_delta);
        /*
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

        //Reset Arm
        slideHW.setSlideServoCurrPos(1, 1 * tilt_arm_pos_delta);
        /*
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

    }

    public void checkGamePad(Gamepad gp) {
        //moveRobotSlideBy(10,0);

        if (gp.dpad_left) {
            slideHW.motorSetRawSpeed(0.95);
            //moveRobotSlideBy(2,0);
        } else if (gp.dpad_right) {
            slideHW.motorSetRawSpeed(-0.95);
            //moveRobotSlideBy(-2,0);

        } else if (gp.dpad_up) {
            dropGameElement();
        }
        /*
        else if (gp.left_bumper) {
            slideHW.pullSlideUp(0.1, false);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        } else if (gp.right_bumper) {
            slideHW.pullSlideUp(0.1, true);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }*/
        else {

            slideHW.motorSetRawSpeed(0.0);
        }

    }


    //For no slide up and down
    public void checkGamePadX(Gamepad gp) {
        if (gp.left_stick_button) {
            //rotate arm
            slideHW.setSlideServoCurrPos(1, -1 * 1.0);
            try {
                Thread.sleep(800);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            slideHW.setSlideServoCurrPos(1, 1 * 1.0);
            try {
                Thread.sleep(800);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        } else if (gp.left_bumper) {
            //rotate arm
            slideHW.setSlideServoCurrPosAbs(1, 1 * 0.3);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (gp.right_bumper) {

            slideHW.setSlideServoCurrPosAbs(1, 1 * 0.0);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

    }

    public void moveSlideToDropPos(int lvl, SlideDirection dir) {
        if (dir == SlideDirection.OUT) {
            if (lvl == 1) {
                //level 1
                moveRobotSlideBy(8, 0);

            } else if (lvl == 2) {
                //level 2
                moveRobotSlideBy(9, -0.1);

            } else {
                //level 3
                moveRobotSlideBy(18, -0.15);
            }
        } else {
            //dir = IN
            if (lvl == 1) {
                //level 1
                moveRobotSlideBy(-8, 0);
            } else if (lvl == 2) {
                //level 2
                moveRobotSlideBy(-9, 0.1);

            } else {
                //level 3
                moveRobotSlideBy(-18, 0.15);
            }
        }

        return;
    }

    private void robotSlidePositionUpdate() {
        /*
        if (tiltSlideBoxPos >= 0) {
            slideHW.setSlideServoCurrPos(0, tiltSlideBoxPos);
            tiltSlideBoxPos = -1.0;
        }

        if (rotateSlideBoxArmPos >= 0) {
            slideHW.setSlideServoCurrPos(1, rotateSlideBoxArmPos);
            rotateSlideBoxArmPos = -1.0;
        }

         */

        if (delta_x != 0 || delta_i != 0 ) {
            //Move the robot Arm only for non-zero values of delta_x, delta_y
            slideHW.moveSlideTo(delta_x);
            slideHW.pullSlideUp(delta_i);
            delta_x = 0;
            delta_i = 0;
            is_done = true;
        }
    }

    public RobotSlideParams getRobotSlideParams() {
        RobotSlideParams param = new RobotSlideParams();
        param.pos = slideHW.getSlideArmInInch();
        param.inclination = slideHW.getSlideCurrIncl();
        param.encoderPosSl1 = slideHW.getSlideEncoderValue(1);
        param.encoderPosSl2 = slideHW.getSlideEncoderValue(2);
        param.servo0Pos = slideHW.getSlideServoCurrPos(0);
        param.servo1Pos = slideHW.getSlideServoCurrPos(1);
        return param;
    }

    public void moveRobotSlideBy(double dx, double di) {
        if (is_done == true) {
            delta_x = dx;
            delta_i = di;
            is_done = false;
        }
    }


    public void stop() {
        isRunning = false;
    }

    @Override
    public void run() {
        while (isRunning) {
            robotSlidePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
