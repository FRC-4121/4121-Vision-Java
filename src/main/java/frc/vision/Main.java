package frc.vision;

import frc.vision.camera.*;
import frc.vision.load.*;
import frc.vision.pipeline.*;
import frc.vision.process.*;
import java.util.Set;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;

public class Main {
    public static void main(String[] args) throws Exception {
        boolean visionDebug = true;

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        CameraLoader.registerFactory(new FrameCamera.Factory());
        CameraLoader.registerFactory(new VideoCaptureCamera.Factory());
        CameraLoader.initConfig();

        ProcessorLoader.registerFactory(new FpsCounter.Factory());
        ProcessorLoader.registerFactory(new AprilTagProcessor.Factory());
        ProcessorLoader.registerFactory(new RectVisionProcessor.Factory());
        ProcessorLoader.initConfig();

        Executor exec = ForkJoinPool.commonPool();

        VisionLibsGroup procs = new VisionLibsGroup(
            ProcessorLoader.loadAll("fps", "april", "ring2024"),
            null, true, exec
        );
        

        ImShower imgs = new ImShower();
        if (visionDebug) {
           procs.setPostProcess(imgs);
        }

        CameraGroup cams = CameraGroup.of("idx0", "dummy");
        cams.setCallback(procs);
        cams.start();

        System.out.println("Running successfully :3");
        
        while (true) {
            if (visionDebug) {
                imgs.run();
                if (imgs.canWaitKey()) {
                    int res = HighGui.waitKey(10);
                    if (res >= 0) {
                        break;
                    }
                }
            }
        }

        cams.cancel();

        HighGui.waitKey(1); // appease opencv
    }
}
