package org.firstinspires.ftc.teamcode.tata.OpenCVDetector;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tata.Common.tataAutonomousBase;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public class OpenCVDetectorHW {

    private OpenCvWebcam webcam;
    private Telemetry telemetry;
    private CapstoneDetector detector = null;


    private tataAutonomousBase.SideColor currSide;

    public void init(HardwareMap ahwMap, tataAutonomousBase.SideColor sc, Telemetry t) {
        telemetry = t;
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id",ahwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(ahwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detector = new CapstoneDetector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        currSide = sc;
    }

    public void stop(){
        webcam.stopStreaming();
    }

    public int getLocation() {
        int markerLoc = 0;
        if (currSide == tataAutonomousBase.SideColor.Red) {
            switch (detector.getLocation()) {
                case pos1:
                    markerLoc = 1;
                    break;
                case pos2:
                    markerLoc = 2;
                    break;
                case pos3:
                    markerLoc = 3;
                    break;
            }

        } else {
            //Blue
            switch (detector.getLocation()) {
                case pos1:
                    markerLoc = 3;
                    break;
                case pos2:
                    markerLoc = 2;
                    break;
                case pos3:
                    markerLoc = 1;
                    break;
            }
        }
        return markerLoc;
    }


}

