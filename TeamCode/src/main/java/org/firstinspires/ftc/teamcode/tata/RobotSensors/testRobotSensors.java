package org.firstinspires.ftc.teamcode.tata.RobotSensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testRobotSensors", group = "Test - TATA")
//@Disabled
public class testRobotSensors extends LinearOpMode {
    public RobotSensorDriver sensorDriver;
    public RobotSensorParams params;

    @Override
    public void runOpMode() {

        sensorDriver = new RobotSensorDriver(hardwareMap, 100);
        Thread sensorDriverThread = new Thread(sensorDriver);
        sensorDriverThread.start();

        double inc = 0.5;

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int barCodeLoc = sensorDriver.getBarCodeRED();
            params = sensorDriver.getRobotSensorParams();
            telemetry.addData("LF:", "%2f inc ", params.x_LF);
            telemetry.addData("LF1:", "%2f inc ", params.x_LF1);
            telemetry.addData("RF:", "%2f inc ", params.x_RF);
            telemetry.addData("RF1:", "%2f inc ", params.x_RF1);
            telemetry.addData("LR:", "%2f inc ", params.x_LR);
            telemetry.addData("RR:", "%2f inc ", params.x_RR);
            telemetry.addData("LS:", "%2f inc ", params.x_LS);
            telemetry.addData("RS:", "%2f inc ", params.x_RS);
            if (params.color == RobotSensorHW.DetectedColors.RED) {
                telemetry.addData("RS:", "RED");
            }
            if (params.color == RobotSensorHW.DetectedColors.WHITE) {
                telemetry.addData("RS:", "WHITE");
            }
            if (params.color == RobotSensorHW.DetectedColors.BLUE) {
                telemetry.addData("RS:", "BLUE");
            }

            telemetry.update();
            sleep(500);
            idle();
        }
        sensorDriver.stop();

        int barCodeLoc = sensorDriver.getBarCodeBLUE();
        params = sensorDriver.getRobotSensorParams();
        telemetry.addData("LF:", "%2f inc ", params.x_LF);
        telemetry.addData("LF1:", "%2f inc ", params.x_LF1);
        telemetry.addData("RF:", "%2f inc ", params.x_RF);
        telemetry.addData("RF1:", "%2f inc ", params.x_RF1);
        telemetry.addData("LR:", "%2f inc ", params.x_LR);
        telemetry.addData("RR:", "%2f inc ", params.x_RR);
        telemetry.addData("Barcode = ", "%2d  ", barCodeLoc);

        telemetry.update();
        sleep(500);
        idle();
        sensorDriver.stop();
    }
}


