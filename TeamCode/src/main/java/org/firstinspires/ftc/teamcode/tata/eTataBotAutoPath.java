package org.firstinspires.ftc.teamcode.tata;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Tata Auto path simple", group="tata")
@Disabled
public class eTataBotAutoPath extends tataBotCommon {

    //@Override
    public void runOpMode() {
        RobotLog.ii("CAL", "Enter  - runOpMode - Esha Autonomous Test");
        initHW(); //initialize hardware

        // Send a telemetry message to signifyrobot waiting;
        waitForStart();

        //run the function that actually moves the robot
        // myDetectionRun(40.0);
        RobotLog.ii("CAL", "Exit - runOpMode - Esha AUtonomous Test");
        park();
    }

    public void park() {
        if (opModeIsActive() && !isStopRequested()){
            robot.dtDriver.setRawPowerAll(-0.5);
            sleep(4000);
            robot.dtDriver.setRawPowerAll(0.0);
        }
    }


    /*public void myDetectionRun(double timeoutS)
    {
        RobotLog.ii("CAL", "Enter i- myDetectionRun");
        //a variable that holds the number of inches we move from the stone closest to the bridge
        //to the skystone so we know how much extra we need to move to always end up the
        //same distance on the other side of the bridge
        double strafe_back_previous = 0;
        //same as above except for the second skystone
        double strafe_back = 0;

        //initialize the motor encoders
        robot.initMotorEncoders();

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {
            //MOve towards the skystones using a distance sensor so we don't collide with them
            myEncoderDrive(Direction.STRAFE_LEFT, 0.2, 50, 5.0, SensorsToUse.USE_DISTANCE_LEFT);
            //align with the reference angle
            correctAngle();

            //if the first stone we see is NOT a skystone, continue to move forward while sensing
            //stop whenever the color sensed is not yellow, but black (skystone)
            //add the extra distance traveled using the color sensor to strafe_back_previous
            if (myDetectSkystone(SideToUse.USE_LEFT, 10) == false) {
                myEncoderDrive(Direction.FORWARD, 0.1, 24, 10.0, SensorsToUse.USE_COLOR_LEFT);
                strafe_back_previous = distance_traveled;
                telemetry.addData("strafe back = ", strafe_back_previous);
                telemetry.update();
                //go backward an inch to be sure that we're aligned with the middle of the skystone
                myEncoderDrive(Direction.BACKWARD, 0.1, 1, 5.0, SensorsToUse.NONE);
            }

            //Grab the skystone and go to the other side of the bridge
            getStone();
            myEncoderDrive(Direction.BACKWARD, DRIVE_SPEED, 35 + strafe_back_previous, 10.0, SensorsToUse.NONE);

            //drop the skystone
            releaseStone();

            //Drive back to collect the second stone
            myEncoderDrive(Direction.FORWARD, DRIVE_SPEED, 52 + strafe_back_previous, 10.0, SensorsToUse.NONE);

            //Drive till we are close to the stone again
            myEncoderDrive(Direction.STRAFE_LEFT, 0.2,24, 5.0, SensorsToUse.USE_DISTANCE_LEFT);

            //move forward while sensing using the color sensor
            //stop whenever the color sensed is not yellow, but black (skystone)
            //add the extra distance traveled using the color sensor to strafe_back
            myEncoderDrive(Direction.FORWARD, 0.1, 20, 5.0, SensorsToUse.USE_COLOR_LEFT);
            strafe_back = distance_traveled;
            telemetry.addData("strafe back = ", strafe_back);
            telemetry.update();
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED,2, 5.0, SensorsToUse.NONE);
            //Grab the skystone
            getStone();
            //second time, we need to strafe an extra inch to avoid the bridge
            myEncoderDrive(Direction.STRAFE_RIGHT, DRIVE_SPEED,3, 5.0, SensorsToUse.NONE);
            correctAngle(); //correct angle to match the reference angle

            //drive to other side and drop the stone
            myEncoderDrive(Direction.BACKWARD, 0.4, 52 + strafe_back_previous + strafe_back, 10.0, SensorsToUse.NONE);
            releaseStone();

            //drive under the bridge then strafe towards the bridge so that our alliance also has space to park
            myEncoderDrive(Direction.FORWARD, 0.4, 15, 10.0, SensorsToUse.NONE);
            myEncoderDrive(Direction.STRAFE_LEFT, DRIVE_SPEED,5, 5.0, SensorsToUse.NONE);
            robot.openCapStoneClaw();

        }
        RobotLog.ii("CAL", "Exit - myDetectionRun");
    }
}*/
}

