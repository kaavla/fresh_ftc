package org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class RobotCarouselHW {

    public CRServo CRS0    = null;
    public CRServo CRS1    = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        CRS0      = ahwMap.get(CRServo.class, "CRS0");
        CRS1      = ahwMap.get(CRServo.class, "CRS1");

    }

    public void setPower(double power){
        CRS0.setPower(power);
        CRS1.setPower(power);
    }

    public double getPower(){
        return CRS0.getPower();
    }

    public void stopCarousel() {
        setPower(0.0);
    }
    public void startCarousel(double power) {
        setPower(power);
    }


}

