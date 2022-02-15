package org.firstinspires.ftc.teamcode.tata.RobotSensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotSensorHW {

    private DistanceSensor dsLF = null; //Left Front
    private DistanceSensor dsRF = null; //Right Front
    private DistanceSensor dsLR = null; //Left Rear
    private DistanceSensor dsRR = null; //Right Rear
    private DistanceSensor dsLS = null; //Left Side
    private DistanceSensor dsRS = null; //Right Side

    private DistanceSensor dsLF1 = null; //Left Front
    private DistanceSensor dsRF1 = null; //Right Front

    public RobotSensorParams rsp = new RobotSensorParams();

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        dsLF = ahwMap.get(DistanceSensor.class, "DSLF");
        dsRF = ahwMap.get(DistanceSensor.class, "DSRF");
        dsLR = ahwMap.get(DistanceSensor.class, "DSLR");
        dsRR = ahwMap.get(DistanceSensor.class, "DSRR");
        dsLS = ahwMap.get(DistanceSensor.class, "DSLS");
        dsRS = ahwMap.get(DistanceSensor.class, "DSRS");

        dsLF1 = ahwMap.get(DistanceSensor.class, "DSLF1");
        dsRF1 = ahwMap.get(DistanceSensor.class, "DSRF1");

    }

    public RobotSensorParams getDistances() {
        rsp.x_LF =dsLF.getDistance(DistanceUnit.INCH);
        rsp.x_RF =dsRF.getDistance(DistanceUnit.INCH);
        rsp.x_LR =dsLR.getDistance(DistanceUnit.INCH);
        rsp.x_RR =dsRR.getDistance(DistanceUnit.INCH);
        rsp.x_LS =dsLS.getDistance(DistanceUnit.INCH);
        rsp.x_RS =dsRS.getDistance(DistanceUnit.INCH);
        rsp.x_LF1 = dsLF1.getDistance(DistanceUnit.INCH);
        rsp.x_RF1 = dsRF1.getDistance(DistanceUnit.INCH);

        return rsp;
    }

}

