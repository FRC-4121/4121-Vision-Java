import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import org.opencv.core.Mat;

// A camera thread that runs the camera as fast as possible, with the option to get futures to the result.
class AsyncCameraThread extends Thread {
    // The camera that we want to loop on.
    CameraBase cam;
    // True if the most recent frame is unprocessed.
    AtomicBoolean ready;
    // A callback that can be run after each frame.
    Consumer<Mat> afterFrame;
    // Queue of futures to be run.
    ConcurrentLinkedQueue<CompletableFuture<Mat>> futures;

    // Create a new thread with the given camera.
    AsyncCameraThread(CameraBase camera) {
        cam = camera;
        ready = new AtomicBoolean();
        afterFrame = _mat -> {};
        futures = new ConcurrentLinkedQueue();
    }

    // Get the callback to be run after each frame.
    public Consumer<Mat> getCallback() {
        return afterFrame;
    }

    // Set a new callback to be run after each frame.
    public void setCallback(Consumer<Mat> callback) {
        afterFrame = callback;
    }

    // Get a future to the most recent frame.
    // May return immediately or add a future to the queue if a frame has already been requested since the last one has been read.
    public CompletableFuture<Mat> getFuture() {
        // this cmpxchg checks to see if there's a ready frame and if so, sets it to false.
        if (ready.compareAndExchange(true, false)) {
            return CompletableFuture.completedFuture(cam.frame.clone());
        }
        // if none was available, add a future to the queue and return it.
        CompletableFuture<Mat> result = new CompletableFuture();
        futures.add(result);
        return result;
    }

    // Run a single frame.
    public void runSingle() {
        cam.run();
        afterFrame.accept(cam.frame);
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
        } while (!future.complete(cam.frame));
    }

    public void run() {
        while (true) runSingle();
    }
}
