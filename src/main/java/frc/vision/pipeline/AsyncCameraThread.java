package frc.vision.pipeline;

import frc.vision.camera.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import org.opencv.core.Mat;

// A camera thread that runs the camera as fast as possible, with the option to get futures to the result.
public class AsyncCameraThread extends Thread {
    // The camera that we want to loop on.
    protected CameraBase cam;
    // True if the most recent frame is unprocessed.
    protected AtomicBoolean ready;
    // A callback that can be run after each frame.
    protected BiConsumer<Mat, ? super CameraBase> afterFrame;
    // Queue of futures to be run.
    protected ConcurrentLinkedQueue<CompletableFuture<Mat>> futures;
    // Whether our loop should be running
    protected boolean running;

    // Create a new thread with the given camera.
    public AsyncCameraThread(CameraBase camera) {
        super(camera.getName());
        cam = camera;
        cam.setCatchExceptions(false); // we want to handle it in our loop
        ready = new AtomicBoolean();
        afterFrame = (_mat, _cam) -> {};
        futures = new ConcurrentLinkedQueue<CompletableFuture<Mat>>();
        running = true;
        setDaemon(true);
    }

    // Set a new callback to be run after each frame.
    public void setSingleCallback(Consumer<Mat> callback) {
        afterFrame = (mat, _cam) -> callback.accept(mat);
    }

    // Set a new callback to be run after each frame.
    public void setCallback(BiConsumer<Mat, ? super CameraBase> callback) {
        afterFrame = callback;
    }

    // Get the camera being run in this thread.
    public CameraBase getCamera() {
        return cam;
    }

    // Politely ask this camera to stop looping and complete.
    public void cancel() {
        running = false;
    }

    // Run a single frame.
    public void runSingle() {
        try {
            cam.run();
            Mat frame = cam.getFrame();
            if (frame == null) return;
            afterFrame.accept(frame, cam);
        } catch (Exception e) {
            e.printStackTrace(cam.getLog());
            if (CameraBase.echoErrors) e.printStackTrace();
            running = false;
        }
    }

    public void run() {
        System.out.println("starting camera uwu");
        while (running) runSingle();
        // running = true;
    }
}
