package frc.vision.pipeline;

import edu.wpi.first.networktables.NetworkTable;
import frc.vision.camera.*;
import frc.vision.process.*;
import java.lang.Void;
import java.util.Collection;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.opencv.core.Mat;

// Vision library group to handle dispatch from a frame to running vision processors.
// Should be fully asynchronous and non-blocking on an input.
public class VisionLibsGroup implements BiConsumer<Mat, CameraBase> {
    static final int IDLE = 0;
    static final int RUNNING = 1;
    static final int READY = 2;

    Executor exec;
    Mat lastFrame;
    CameraBase lastHandle;
    Collection<? extends VisionProcessor> procs;
    NetworkTable table;
    BiConsumer<Mat, ? super CameraBase> postProcess;
    CompletableFuture<Void> handle;
    AtomicInteger state;
    boolean visionDebug;

    public VisionLibsGroup(Collection<? extends VisionProcessor> procs, NetworkTable table, boolean visionDebug, Executor exec) {
        this.procs = procs;
        this.exec = exec;
        this.table = table;
        this.visionDebug = visionDebug;

        state = new AtomicInteger(IDLE);
        postProcess = (_mat, _h) -> {};
    }

    public void setPostProcess(BiConsumer<Mat, ? super CameraBase> callback) {
        postProcess = callback;
    }

    public void accept(Mat frame, CameraBase handle) {
        lastFrame = frame;
        lastHandle = handle;
        if (state.compareAndSet(IDLE, RUNNING)) {
            scheduleSelf();
        } else {
            state.set(READY);
        }
    }
    protected Stream<? extends VisionProcessor> getLibs(Collection<String> vlibs) {
        return procs.stream().filter(proc -> vlibs == null || vlibs.size() == 0 || vlibs.contains(proc.getName()));
    }
    protected void scheduleSelf() {
        Mat frame = lastFrame;
        CameraBase cam = lastHandle;
        CompletableFuture<Void> future = CompletableFuture.allOf(
            getLibs(cam.getConfig().vlibs)
                .map(proc -> CompletableFuture.runAsync(() -> proc.process(frame, cam.getConfig(), handle), exec))
                .toArray(size -> new CompletableFuture[size])
        );
        if (table != null || visionDebug) {
            future = future.thenCompose(_void -> {
                Stream<CompletableFuture<Void>> tables = Stream.empty();
                if (table != null) {
                    NetworkTable subTable = table.getSubTable(cam.getName());
                    tables = getLibs(cam.getConfig().vlibs)
                        .map(proc -> CompletableFuture.runAsync(() -> proc.toNetworkTable(table, cam), exec));
                }
                Stream<CompletableFuture<Void>> drawings = !visionDebug
                    ? Stream.empty()
                    : getLibs(cam.getConfig().vlibs)
                        .map(proc -> CompletableFuture.runAsync(() -> proc.drawOnImage(frame, cam), exec));
                return CompletableFuture.allOf(Stream.concat(tables, drawings).toArray(size -> new CompletableFuture[size]));
            });
        }
        handle = future
            .thenAcceptAsync(_void -> postProcess.accept(frame, cam), exec)
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
