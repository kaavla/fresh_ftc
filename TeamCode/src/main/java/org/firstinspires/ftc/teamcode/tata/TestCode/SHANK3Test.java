package org.firstinspires.ftc.teamcode.tata.TestCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.tata.RobotArm.RobotArmDriver;
import org.firstinspires.ftc.teamcode.tata.RobotSensors.RobotSensorParams;

@Autonomous(name = "RED - SHANK3 - AllianceStorage", group = "RED")
@Disabled
public class SHANK3Test extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-42.25, -63.5, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, startPose);

        waitForStart();

        if (isStopRequested()) {
            stopThreads();
            return;
        }

        //Move Robot Arm to collect pos

        armDriver.moveRobotArmTo(RobotArmDriver.RobotArmPreSetPos.COLLECT); //collect position

        //telemetry.addData("Rel:", "%2f deg ", imuParams.correctedHeading);
        //telemetry.addData("diff:", "%2f deg ", Math.abs(imuParams.correctedHeading - 270));
        //telemetry.update();
        sleep(6000);

        stopThreads();

    }
}
