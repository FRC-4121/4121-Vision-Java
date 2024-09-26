package frc.vision.pipeline;

import frc.vision.camera.CameraBase;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BiConsumer;
import org.opencv.core.Mat;
import org.opencv.highgui.HighGui;

// Wrapper to handle the multi-threaded stuff for imshow
public class ImShower implements BiConsumer<Mat, CameraBase>, Runnable {
    protected ConcurrentHashMap<String, Mat> frames;
    protected boolean calledOnce;

    public ImShower() {
        frames = new ConcurrentHashMap();
        calledOnce = false;
    }

    // Accept a frame with the given name
    @Override
    public void accept(Mat frame, CameraBase cam) {
        frames.put(cam.getName(), frame);
    }

    // Show all of the frames
    @Override
    public void run() {
        frames.forEach((name, frame) -> {
            HighGui.imshow(name, frame);
            calledOnce = true;
        });
    }

    public boolean canWaitKey() {
        return calledOnce;
    }
}
