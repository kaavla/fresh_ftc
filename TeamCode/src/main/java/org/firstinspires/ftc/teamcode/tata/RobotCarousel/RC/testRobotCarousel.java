package org.firstinspires.ftc.teamcode.tata.RobotCarousel.RC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;

@TeleOp(name = "testRobotCarousel", group = "Test - TATA")
//@Disabled
public class testRobotCarousel extends LinearOpMode {
    public RobotCaroselDriver driver;

    private double motor_power = 0.9;
    @Override
    public void runOpMode() {
        driver = new RobotCaroselDriver(hardwareMap, 200, tataAutonomousBase.SideColor.Blue);
        Thread driverThread = new Thread(driver);
        driverThread.start();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driver.checkGamePad(gamepad1);
            sleep(50);
            idle();
        }
        driver.stop();
    }
}

