package frc.vision.pipeline;

import edu.wpi.first.networktables.NetworkTable;
import frc.vision.camera.*;
import frc.vision.process.*;
import java.lang.Void;
import java.util.Collection;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import java.util.function.BiConsumer;
import java.util.stream.Stream;
import org.opencv.core.Mat;

// Vision library group to handle dispatch from a frame to running vision processors.
// Should be mostly non-blocking.
public class VisionLibsGroup implements BiConsumer<Mat, CameraBase> {
    public static final int MAX_QUEUE = 4;
    public static final int MAX_PROCS = 2;
    protected static class CamState {
        ConcurrentHashMap<CompletableFuture<Void>, Integer> handles;
        RingBuffer<Mat> frames;
        AtomicInteger running;
        boolean loggedLibs;

        public CamState() {
            handles = new ConcurrentHashMap<>();
            frames = new RingBuffer<>(MAX_QUEUE);
            running = new AtomicInteger();
            loggedLibs = false;
        }
    }
    Executor exec;
    Collection<VisionProcessor> procs;
    NetworkTable table;
    BiConsumer<Mat, ? super CameraBase> postProcess;
    ConcurrentHashMap<CameraBase, CamState> states;
    boolean visionDebug;

    public VisionLibsGroup(Collection<VisionProcessor> procs, NetworkTable table, boolean visionDebug, Executor exec) {
        this.procs = procs;
        this.exec = exec;
        this.table = table;
        this.visionDebug = visionDebug;

        states = new ConcurrentHashMap<CameraBase, CamState>();
    }

    public void setPostProcess(BiConsumer<Mat, ? super CameraBase> callback) {
        postProcess = callback;
    }

    private CamState getState(CameraBase cam) {
        CamState state = states.putIfAbsent(cam, new CamState());
        if (state == null) state = states.get(cam);
        return state;
    }

    @Override
    public void accept(Mat frame, CameraBase cam) {
        if (frame == null) return;
        if (frame.dataAddr() == 0) return;
        CamState state = getState(cam);
        state.frames.add(frame.clone());
        scheduleSelf(cam, state);
    }
    public Stream<VisionProcessor> getLibs(Collection<String> vlibs) {
        return procs.stream().filter(proc -> vlibs == null || vlibs.size() == 0 || vlibs.contains(proc.getName()));
    }
    public void add(VisionProcessor proc) {
        procs.add(proc);
    }
    protected void scheduleSelf(CameraBase cam, CamState state) {
        if (!state.loggedLibs) {
            String names = getLibs(cam.getConfig().vlibs)
                .map(p -> p.getName())
                .reduce((a, b) -> a + ", " + b)
                .orElse("<none>");
            cam.getLog().write(String.format("Using processors: %s\n", names));
            cam.getLog().flush();
            state.loggedLibs = true;
        }
        if (state.running.getAndIncrement() >= MAX_PROCS) {
            state.running.decrementAndGet();
            return;
        }
        Mat frame = state.frames.remove();
        if (frame == null) {
            state.running.decrementAndGet();
            return;
        }
        CompletableFuture<Void> future = CompletableFuture.allOf(
            getLibs(cam.getConfig().vlibs)
                .map(proc -> CompletableFuture.runAsync(() -> proc.process(frame, cam), exec))
                .toArray(size -> new CompletableFuture[size])
        );
        if (table != null || visionDebug) {
            future = future.thenCompose(_void -> {
                Stream<CompletableFuture<Void>> tables = Stream.empty();
                if (table != null) {
                    NetworkTable subTable = table.getSubTable(cam.getName());
                    tables = getLibs(cam.getConfig().vlibs)
                        .map(proc -> CompletableFuture.runAsync(() -> proc.toNetworkTable(subTable, cam), exec));
                }
                Stream<CompletableFuture<Void>> drawings = !visionDebug
                    ? Stream.empty()
                    : getLibs(cam.getConfig().vlibs)
                        .map(proc -> CompletableFuture.runAsync(() -> proc.drawOnImage(frame, cam), exec));
                return CompletableFuture.allOf(Stream.concat(tables, drawings).toArray(size -> new CompletableFuture[size]));
            });
        }
        RecursiveFutureRemover cleanup = new RecursiveFutureRemover();
        cleanup.cam = cam;
        cleanup.state = state;
        CompletableFuture fut = future
            .thenRunAsync(() -> postProcess.accept(frame, cam), exec)
            .exceptionally(e -> {
                e.printStackTrace(cam.getLog());
                return null;
            })
            .whenComplete(cleanup);
        cleanup.handle = fut;
        state.handles.put(fut, 0);
    }
    public void cancel() {
        for (CamState state : states.values()) {
            for (CompletableFuture<Void> handle : state.handles.keySet()) handle.cancel(false);
        }
    }
    public boolean isRunning() {
        for (CamState state : states.values()) {
            for (CompletableFuture<Void> handle : state.handles.keySet()) if (!handle.isDone()) return true;
        }
        return false;
    }

    private class RecursiveFutureRemover implements BiConsumer<Object, Object> {
        CameraBase cam;
        CamState state;
        CompletableFuture<Void> handle;

        @Override
        public void accept(Object _void, Object _ex) {
            if (handle != null) state.handles.remove(handle);
            state.running.decrementAndGet();
            scheduleSelf(cam, state);
        }
    }
}
