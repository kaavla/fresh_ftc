package org.firstinspires.ftc.teamcode.tata.OpenCVDetector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC.RobotCaroselDriver;

@TeleOp(name = "testOpenCVDetector", group = "Test - TATA")
//@Disabled
public class testOpenCVDetector extends LinearOpMode {
    public OpenCVDetectorDriver driver;
    public int markerPos = 0;

    @Override
    public void runOpMode() {
        driver = new OpenCVDetectorDriver(hardwareMap, 200, tataAutonomousBase.SideColor.Blue, telemetry);
        Thread driverThread = new Thread(driver);
        driverThread.start();

        // Wait for the game to begin
        while (!isStopRequested() && !isStarted()) {
            markerPos = driver.getMarkerPos();

        }
        driver.stop();


        waitForStart();
        telemetry.addData(">", "Marker Found %d", markerPos);
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        sleep(10000);
        idle();
    }
}

