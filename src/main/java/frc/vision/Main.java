package frc.vision;

import frc.vision.camera.*;
import frc.vision.pipeline.*;
import frc.vision.process.*;
import java.util.Set;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;

public class Main {
    static class QueuedImage {
        public String name;
        public Mat frame;
        public QueuedImage(String name, Mat frame) {
            this.name = name;
            this.frame = frame;
        }
    }

    public static void main(String[] args) throws Exception {
        boolean visionDebug = true;

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        CameraLoader.registerFactory(new FrameCamera.Factory());
        CameraLoader.initConfig();

        Executor exec = ForkJoinPool.commonPool();

        VisionLibsGroup group = new VisionLibsGroup(Set.of(new FpsCounter()), null, true, exec);
        
        final ConcurrentLinkedQueue<QueuedImage> imgQueue = new ConcurrentLinkedQueue();

        if (visionDebug) {
            group.setPostProcess((frame, cam) -> imgQueue.add(new QueuedImage(cam.getName(), frame)));
        }

        AsyncCameraThread dummy = new AsyncCameraThread(CameraLoader.load("dummy"));
        dummy.setCallback(group);

        dummy.start();

        System.out.println("Running successfully :3");
        

        QueuedImage img;
        boolean running = true;
        int count = 0;
        while (running) {
            if (visionDebug) {
                while ((img = imgQueue.poll()) != null) {
                    HighGui.imshow(img.name, img.frame);
                    int res = HighGui.waitKey(10);
                    if (res >= 0) {
                        running = false;
                        break;
                    }
                }
            }
        }

        dummy.cancel();

        HighGui.waitKey(1); // appease opencv
    }
}
