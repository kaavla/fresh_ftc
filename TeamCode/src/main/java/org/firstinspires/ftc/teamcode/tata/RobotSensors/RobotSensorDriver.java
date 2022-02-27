package org.firstinspires.ftc.teamcode.tata.RobotSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotSensorDriver implements Runnable {
    private RobotSensorHW sensorHW = new RobotSensorHW();

    private RobotSensorParams rsp = new RobotSensorParams();

    private static final int MAX_FILTER_SIZE = 1;
    private RobotSensorParams[] rspAray = new RobotSensorParams[MAX_FILTER_SIZE];
    private int arrIdx = 0;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public RobotSensorDriver(HardwareMap ahwMap, int threadSleepDelay) {
        sensorHW.init(ahwMap);
        sleepTime = threadSleepDelay;
    }

    public int getBarCodeRED() {
        RobotSensorParams avg_rsp = getAvg();

        int barCodeLoc = 1;
        //check the left front sensors
        if ((avg_rsp.x_LF < 20) || (avg_rsp.x_LF1 < 20)) {
            barCodeLoc = 1;
        } else if ((avg_rsp.x_RF < 20) || (avg_rsp.x_RF1 < 20)) {
            barCodeLoc = 2;
        } else {
            barCodeLoc = 3;
        }
        return barCodeLoc;
    }
    public int getBarCodeBLUE() {
        RobotSensorParams avg_rsp = getAvg();
        int barCodeLoc = 1;
        //check the left front sensors
        if ((avg_rsp.x_RF < 10) || (avg_rsp.x_RF1 < 10)) { // params were 20
            barCodeLoc = 1;
        } else if ((avg_rsp.x_LF < 10) || (avg_rsp.x_LF1 < 10)) { // params were 20
            barCodeLoc = 2;
        } else {
            barCodeLoc = 3;
        }
        return barCodeLoc;
    }

    private void robotSensorUpdate() {
        rsp = sensorHW.getDistances();
        addToFilter(rsp);
    }

    public RobotSensorParams getRobotSensorParams(){
        rsp = getAvg();
        return rsp;
    }

    public void addToFilter(RobotSensorParams in) {
        rspAray[arrIdx] = in;
        /*
        rspAray[arrIdx].x_LF = in.x_LF;
        rspAray[arrIdx].x_RF = in.x_RF;
        rspAray[arrIdx].x_LR = in.x_LR;
        rspAray[arrIdx].x_RR = in.x_RR;
        rspAray[arrIdx].x_LF1 = in.x_LF1;
        rspAray[arrIdx].x_RF1 = in.x_RF1;
        rspAray[arrIdx].x_LS = in.x_LS;
        rspAray[arrIdx].x_RS = in.x_RS;
         */

        arrIdx = arrIdx + 1;
        if (arrIdx >= MAX_FILTER_SIZE) {
            arrIdx = 0;
        }
    }

    public RobotSensorParams getAvg(){
        RobotSensorParams avg = new RobotSensorParams();
        avg.x_LF = 0;
        avg.x_RF = 0;
        avg.x_LR = 0;
        avg.x_RR = 0;
        avg.x_LF1 = 0;
        avg.x_RF1 = 0;
        avg.x_LS = 0;
        avg.x_RS = 0;
        for (int i = 0; i < MAX_FILTER_SIZE; i++) {
            avg.x_LF += rspAray[i].x_LF;
            avg.x_RF += rspAray[i].x_RF;
            avg.x_LR += rspAray[i].x_LR;
            avg.x_RR += rspAray[i].x_RR;
            avg.x_LF1 += rspAray[i].x_LF1;
            avg.x_RF1 += rspAray[i].x_RF1;
            avg.x_LS += rspAray[i].x_LS;
            avg.x_RS += rspAray[i].x_RS;
        }
        avg.x_LF = rsp.x_LF/MAX_FILTER_SIZE;
        avg.x_RF = rsp.x_RF/MAX_FILTER_SIZE;
        avg.x_LR = rsp.x_LR/MAX_FILTER_SIZE;
        avg.x_RR = rsp.x_RR/MAX_FILTER_SIZE;
        avg.x_LF1 = rsp.x_LF1/MAX_FILTER_SIZE;
        avg.x_RF1 = rsp.x_RF1/MAX_FILTER_SIZE;
        avg.x_LS = rsp.x_LS/MAX_FILTER_SIZE;
        avg.x_RS = rsp.x_RS/MAX_FILTER_SIZE;

        return avg;
    }


    public void stop() {
        isRunning = false;
    }

    @Override
    public void run() {
        while (isRunning) {
            robotSensorUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
