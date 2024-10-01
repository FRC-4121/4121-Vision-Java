package frc.vision.pipeline;

import edu.wpi.first.networktables.NetworkTable;
import frc.vision.camera.*;
import frc.vision.process.*;
import java.lang.Void;
import java.time.*;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.opencv.core.Mat;

// Vision library group to handle dispatch from a frame to running vision processors.
// Should be mostly non-blocking.
public class VisionLibsGroup implements BiConsumer<Mat, CameraBase> {
    protected static class CamState {
        CompletableFuture<Boolean> handle;
        Mat frame;
        AtomicBoolean ready;

        public CamState() {
            handle = CompletableFuture.completedFuture(false);
            frame = new Mat();
            ready = new AtomicBoolean(false);
        }
    }
    Executor exec;
    Mat lastFrame;
    CameraBase lastHandle;
    Collection<? extends VisionProcessor> procs;
    NetworkTable table;
    BiConsumer<Mat, ? super CameraBase> postProcess;
    CompletableFuture<Boolean> handle;
    IdentityHashMap<CameraBase, CamState> states;
    boolean visionDebug;

    AtomicInteger running;


    public VisionLibsGroup(Collection<? extends VisionProcessor> procs, NetworkTable table, boolean visionDebug, Executor exec) {
        this.procs = procs;
        this.exec = exec;
        this.table = table;
        this.visionDebug = visionDebug;

        states = new IdentityHashMap();
        running = new AtomicInteger(0);
    }

    public void setPostProcess(BiConsumer<Mat, ? super CameraBase> callback) {
        postProcess = callback;
    }

    public VisionLibsGroup clone() {
        return new VisionLibsGroup(procs, table, visionDebug, exec);
    }

    private CamState getState(CameraBase cam) {
        CamState state = states.putIfAbsent(cam, new CamState());
        if (state == null) state = states.get(cam);
        return state;
    }

    @Override
    public void accept(Mat frame, CameraBase cam) {
        CamState state = getState(cam);
        synchronized(state) {
            boolean running = state.handle.getNow(false);
            state.frame = frame;
            if (!running) scheduleSelf(cam, state);
            else state.ready.set(true);
        }
    }
    protected Stream<? extends VisionProcessor> getLibs(Collection<String> vlibs) {
        return procs.stream().filter(proc -> vlibs == null || vlibs.size() == 0 || vlibs.contains(proc.getName()));
    }
    protected void scheduleSelf(CameraBase cam, CamState state) {
        synchronized(state) {
            if (!state.handle.isDone()) return;
            Mat frame = state.frame.clone();
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
            state.handle = future
                .thenRunAsync(() -> postProcess.accept(frame, cam), exec)
                .thenRunAsync(() -> {
                    if (state.ready.compareAndSet(true, false)) scheduleSelf(cam, state);
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
