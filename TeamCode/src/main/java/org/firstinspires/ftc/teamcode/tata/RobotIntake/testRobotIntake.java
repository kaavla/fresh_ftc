package org.firstinspires.ftc.teamcode.tata.RobotIntake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tata.RobotSlide.RobotSlideDriver;

@TeleOp(name = "testRobotIntake", group = "Test - TATA")
//@Disabled
public class testRobotIntake extends LinearOpMode {
    public RobotIntakeDriver driver;

    private double motor_power = 1.0;
    @Override
    public void runOpMode() {
        driver = new RobotIntakeDriver(hardwareMap, 200);
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

