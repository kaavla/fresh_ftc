package org.firstinspires.ftc.teamcode.tata.RobotDrivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tata.RobotIntake.RobotIntakeDriver;


@TeleOp(name = "testRobotDrivetrain", group = "Test - TATA")
//@Disabled
public class testRobotDrivetrain extends LinearOpMode {
    public RobotDriveTrainDriver dtDriver ;
    //public RobotDrivetrainHW dt = new RobotDrivetrainHW();
    //private float leftX, leftY, rightZ;
    //private double motor_power = 0.3;
    public RobotDrivetrainParams params;

    @Override
    public void runOpMode() {
        dtDriver = new RobotDriveTrainDriver(hardwareMap, 500);
        Thread driverThread = new Thread(dtDriver);
        driverThread.start();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            dtDriver.checkGamePad(gamepad1);

            params = dtDriver.getRobotDrivetrainParams();
            telemetry.addData("LF:", "%2d inc ", params.enc_LF);
            telemetry.addData("RF:", "%2d inc ", params.enc_RF);
            telemetry.addData("LR:", "%2d inc ", params.enc_LR);
            telemetry.addData("RR:", "%2d inc ", params.enc_RR);
            telemetry.update();

            sleep(50);
            idle();
        }
        dtDriver.stop();
    }
}

