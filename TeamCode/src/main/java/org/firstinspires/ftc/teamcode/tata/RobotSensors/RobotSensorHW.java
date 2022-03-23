package org.firstinspires.ftc.teamcode.tata.RobotSensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotSensorHW {
    public enum DetectedColors{
        RED,
        BLUE,
        WHITE
    }

    private DistanceSensor dsLF = null; //Left Front
    private DistanceSensor dsRF = null; //Right Front
    private DistanceSensor dsLR = null; //Left Rear
    private DistanceSensor dsRR = null; //Right Rear
    private DistanceSensor dsLS = null; //Left Side
    private DistanceSensor dsRS = null; //Right Side

    //private DistanceSensor dsLF1 = null; //Left Front
    //private DistanceSensor dsRF1 = null; //Right Front
    private ColorSensor csB = null;

    public RobotSensorParams rsp = new RobotSensorParams();

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        dsLF = ahwMap.get(DistanceSensor.class, "DSLF");
        dsRF = ahwMap.get(DistanceSensor.class, "DSRF");
        dsLR = ahwMap.get(DistanceSensor.class, "DSLR");
        dsRR = ahwMap.get(DistanceSensor.class, "DSRR");
        dsLS = ahwMap.get(DistanceSensor.class, "DSLS");
        dsRS = ahwMap.get(DistanceSensor.class, "DSRS");
        //csB  = ahwMap.get(ColorSensor.class, "CSB");
        //dsLF1 = ahwMap.get(DistanceSensor.class, "DSLF1");
        //dsRF1 = ahwMap.get(DistanceSensor.class, "DSRF1");

    }

    public RobotSensorParams getDistances() {
        rsp.x_LF =dsLF.getDistance(DistanceUnit.INCH);
        rsp.x_RF =dsRF.getDistance(DistanceUnit.INCH);
        rsp.x_LR =dsLR.getDistance(DistanceUnit.INCH);
        rsp.x_RR =dsRR.getDistance(DistanceUnit.INCH);
        rsp.x_LS =dsLS.getDistance(DistanceUnit.INCH);
        rsp.x_RS =dsRS.getDistance(DistanceUnit.INCH);
        //rsp.x_LF1 = dsLF1.getDistance(DistanceUnit.INCH);
        //rsp.x_RF1 = dsRF1.getDistance(DistanceUnit.INCH);
        //rsp.color = detectColor();

        return rsp;
    }

/*
    public DetectedColors detectColor() {
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;


            Color.RGBToHSV((int) (csB.red() * SCALE_FACTOR),
                    (int) (csB.green() * SCALE_FACTOR),
                    (int) (csB.blue() * SCALE_FACTOR),
                    hsvValues);



        //RED
        if ((hsvValues[0] > 0 && hsvValues[0] < 20) || (hsvValues[0] > 350 && hsvValues[0] < 360)) {
            return DetectedColors.RED;
        }
        if (hsvValues[0] > 220 && hsvValues[0] < 260) {
            return DetectedColors.BLUE;
        }
        if ((hsvValues[0] >= 0 && hsvValues[0] < 170) &&
           (hsvValues[1] >= 0 && hsvValues[1] < 110) &&
           (hsvValues[2] > 170 && hsvValues[2] < 255)){
            return DetectedColors.WHITE;
        }
        return DetectedColors.RED;

    }

*/

}

