package frc.vision;

import frc.vision.camera.*;
import frc.vision.pipeline.*;
import frc.vision.process.*;
import java.util.ArrayList;
import java.util.concurrent.*;
import org.opencv.core.Core;
import org.opencv.highgui.HighGui;

public class Main {
    public static void main(String[] args) throws Exception {
        boolean visionDebug = true;

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        CameraLoader.registerFactory(new FrameCamera.Factory());
        CameraLoader.initConfig();

        Executor exec = ForkJoinPool.commonPool();

        ArrayList<VisionProcessor> libs = new ArrayList();

        VisionLibsGroup group = new VisionLibsGroup(libs, null, true, exec);
        if (visionDebug) {
            group.setPostProcess((frame, cam) -> {
                HighGui.imshow(cam.getName(), frame);
            });
        }

        AsyncCameraThread dummy = new AsyncCameraThread(CameraLoader.load("dummy"));
        dummy.setCallback(group);

        dummy.start();

        System.out.println("Running successfully :3");

        while (true) {}
    }
}
