package org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;


public class RobotCarouselHW {

    public CRServo CRS0    = null;
    public CRServo CRS1    = null;
    public tataAutonomousBase.SideColor currSide;
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, tataAutonomousBase.SideColor sc) {

        CRS0      = ahwMap.get(CRServo.class, "CRS0");
        CRS1      = ahwMap.get(CRServo.class, "CRS1");
        currSide = sc;

    }
    public void reverseDir() {
        if (currSide == tataAutonomousBase.SideColor.Blue) {
            currSide = tataAutonomousBase.SideColor.Red;
        } else {
            currSide = tataAutonomousBase.SideColor.Blue;
        }

    }

    public void setPower(double power){
        if (currSide == tataAutonomousBase.SideColor.Blue) {
            CRS0.setPower(power);
            CRS1.setPower(power);
        }else {
            CRS0.setPower(-1*power);
            CRS1.setPower(-1*power);
        }
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

