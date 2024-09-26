package frc.vision.pipeline;

import edu.wpi.first.networktables.NetworkTable;
import frc.vision.camera.*;
import frc.vision.process.*;
import java.lang.Void;
import java.util.Collection;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.opencv.core.Mat;

// Vision library group to handle dispatch from a frame to running vision processors.
// Should be mostly non-blocking.
public class VisionLibsGroup implements BiConsumer<Mat, CameraBase> {
    Executor exec;
    Mat lastFrame;
    CameraBase lastHandle;
    Collection<? extends VisionProcessor> procs;
    NetworkTable table;
    BiConsumer<Mat, ? super CameraBase> postProcess;
    CompletableFuture<Boolean> handle;
    AtomicBoolean ready;
    boolean visionDebug;

    public VisionLibsGroup(Collection<? extends VisionProcessor> procs, NetworkTable table, boolean visionDebug, Executor exec) {
        this.procs = procs;
        this.exec = exec;
        this.table = table;
        this.visionDebug = visionDebug;

        ready = new AtomicBoolean(false);
        postProcess = (_mat, _h) -> {};
        handle = CompletableFuture.completedFuture(false);
    }

    public void setPostProcess(BiConsumer<Mat, ? super CameraBase> callback) {
        postProcess = callback;
    }

    public void accept(Mat frame, CameraBase camHandle) {
        synchronized(handle) {
            boolean running = false;
            if (handle != null) {
                running = handle.getNow(true);
            }
            lastFrame = frame;
            lastHandle = camHandle;
            if (!running) scheduleSelf();
            else ready.set(true);
        }
    }
    protected Stream<? extends VisionProcessor> getLibs(Collection<String> vlibs) {
        return procs.stream().filter(proc -> vlibs == null || vlibs.size() == 0 || vlibs.contains(proc.getName()));
    }
    protected void scheduleSelf() {
        synchronized(handle) {
            Mat frame = lastFrame;
            CameraBase cam = lastHandle;
            CompletableFuture<Void> future = CompletableFuture.allOf(
                getLibs(cam.getConfig().vlibs)
                    .map(proc -> CompletableFuture.runAsync(() -> proc.process(frame, cam.getConfig(), cam), exec))
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
            handle.cancel(true);
            handle = future
                .thenRunAsync(() -> postProcess.accept(frame, cam), exec)
                .thenRunAsync(() -> {
                    if (ready.compareAndSet(true, false)) scheduleSelf();
                }, exec)
                .thenApply(_void -> false);
        }
    }
    public void cancel() {
        handle.cancel(false);
    }
    public boolean isRunning() {
        return handle != null;
    }
}
