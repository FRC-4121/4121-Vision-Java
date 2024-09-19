import edu.wpi.first.networktables.NetworkTable;
import java.lang.Void;
import java.util.Collection;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.opencv.core.Mat;

// Vision library group to handle dispatch from a frame to running vision processors.
// Should be fully asynchronous and non-blocking on an input.
class VisionLibsGroup implements Consumer<Mat> {
    static final int IDLE = 0;
    static final int RUNNING = 1;
    static final int READY = 2;

    Executor exec;
    CameraConfig cfg;
    Mat lastFrame;
    Collection<VisionProcessor> procs;
    NetworkTable table;
    Consumer<Mat> postProcess;
    CompletableFuture<Void> handle;
    AtomicInteger state;
    boolean visionDebug;

    public VisionLibsGroup(CameraConfig cfg, Collection<VisionProcessor> procs, NetworkTable table, boolean visionDebug, Executor exec) {
        this.cfg = cfg;
        this.procs = procs;
        this.exec = exec;
        this.table = table;
        this.visionDebug = visionDebug;

        state = new AtomicInteger(IDLE);
        postProcess = _mat -> {};
    }

    public void setPostProcess(Consumer<Mat> callback) {
        postProcess = callback;
    }

    public void accept(Mat frame) {
        lastFrame = frame;
        if (state.compareAndSet(IDLE, RUNNING)) {
            scheduleSelf();
        } else {
            state.set(READY);
        }
    }
    protected Stream<VisionProcessor> getLibs() {
        return procs.stream().filter(proc -> cfg.vlibs.size() == 0 || cfg.vlibs.contains(proc.getName()));
    }
    protected void scheduleSelf() {
        Mat frame = lastFrame;
        CompletableFuture<Void> future = CompletableFuture.allOf(
            getLibs()
                .map(proc -> CompletableFuture.runAsync(() -> proc.process(frame, cfg), exec))
                .toArray(size -> new CompletableFuture[size])
        );
        if (table != null || visionDebug) {
            future = future.thenCompose(_void -> {
                Stream<CompletableFuture<Void>> tables = table == null
                    ? Stream.empty()
                    : getLibs()
                        .map(proc -> CompletableFuture.runAsync(() -> proc.toNetworkTable(table), exec));
                Stream<CompletableFuture<Void>> drawings = !visionDebug
                    ? Stream.empty()
                    : getLibs()
                        .map(proc -> CompletableFuture.runAsync(() -> proc.drawOnImage(frame), exec));
                Stream<CompletableFuture<Void>> combined = Stream.concat(tables, drawings);
                return CompletableFuture.allOf(Stream.concat(tables, drawings).toArray(size -> new CompletableFuture[size]));
            });
        }
        handle = future
            .thenApply(_void -> frame)
            .thenAcceptAsync(postProcess, exec)
            .thenRun(() -> {
                synchronized(this) {
                    handle = null;
                }
                if (state.compareAndSet(READY, RUNNING)) {
                    scheduleSelf();
                } else {
                    state.set(IDLE);
                }
            });
    }
    public void cancel() {
        synchronized(this) {
            if (handle != null) handle.cancel(false);
        }
    }
    public boolean isRunning() {
        return handle != null;
    }
}
