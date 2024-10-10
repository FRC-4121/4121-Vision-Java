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
    // The last seen frame
    protected Mat lastFrame;

    // Create a new thread with the given camera.
    public AsyncCameraThread(CameraBase camera) {
        super(camera.getName());
        cam = camera;
        cam.setCatchExceptions(false); // we want to handle it in our loop
        ready = new AtomicBoolean();
        afterFrame = (_mat, _cam) -> {};
        futures = new ConcurrentLinkedQueue();
        running = true;
        lastFrame = new Mat();
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

    // Get a future to the most recent frame.
    // May return immediately or add a future to the queue if a frame has already been requested since the last one has been read.
    public CompletableFuture<Mat> getFuture() {
        // this cmpxchg checks to see if there's a ready frame and if so, sets it to false.
        if (ready.compareAndExchange(true, false)) {
            return CompletableFuture.completedFuture(cam.getFrame().clone());
        }
        // if none was available, add a future to the queue and return it.
        CompletableFuture<Mat> result = new CompletableFuture();
        futures.add(result);
        return result;
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
            frame.copyTo(lastFrame);
            afterFrame.accept(lastFrame, cam);
            // After we've got a frame, we see if a future's waiting.
            CompletableFuture<Mat> future = null;
            do {
                future = futures.poll();
                // If no futures are available, then we just say that we're ready for a new frame.
                if (future == null) {
                    ready.setRelease(true);
                    return;
                }
                // try to complete the future, but if it was cancelled or already completed for some other reason, we loop and check for the next one.
            } while (!future.complete(lastFrame));
        } catch (Exception e) {
            e.printStackTrace(cam.getLog());
            if (CameraBase.echoErrors) e.printStackTrace();
            running = false;
        }
    }

    public void run() {
        while (running) runSingle();
        // running = true;
    }
}
