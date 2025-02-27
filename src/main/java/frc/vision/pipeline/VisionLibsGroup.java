package frc.vision.pipeline;

import edu.wpi.first.networktables.NetworkTable;
import frc.vision.camera.*;
import frc.vision.process.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import java.util.function.BiConsumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.opencv.core.Mat;

// Vision library group to handle dispatch from a frame to running vision processors.
// Should be mostly non-blocking.
public class VisionLibsGroup implements BiConsumer<Mat, CameraBase> {
    public static final int MAX_QUEUE = 1;
    public static final int MAX_PROCS = 1; // I want real framerate
    protected class CamState {
        ConcurrentHashMap<CompletableFuture<Void>, Integer> handles;
        RingBuffer frames;
        AtomicInteger running;
        ArrayList<ArrayList<Integer>> plan;
        boolean loggedLibs;

        public CamState() {
            handles = new ConcurrentHashMap<>();
            frames = new RingBuffer(MAX_QUEUE);
            running = new AtomicInteger();
            loggedLibs = false;
        }

        public void buildPlan(Collection<String> vlibs) {
            if (plan == null) rebuildPlan(vlibs);
        }

        public void rebuildPlan(Collection<String> vlibs) {
            class Proc {
                int index;
                Collection<String> deps;
                Collection<Integer> resDeps;

                Proc(int index) {
                    this.index = index;
                }
                Proc(int index, Collection<String> deps) {
                    this.index = index;
                    this.deps = deps;
                }
            }
            plan = new ArrayList(1);
            HashMap<String, Proc> map = new HashMap<>();
            HashSet<Integer> seen = new HashSet<>();
            int idx = 0;
            boolean again = false;

            for (VisionProcessor proc : procs) {
                ProcessorConfig cfg = proc.getConfig();
                if (vlibs != null && !vlibs.contains(proc.getName())) {
                    seen.add(idx);
                    map.put(proc.getName(), new Proc(idx));
                } else if (cfg == null || cfg.deps == null || cfg.deps.size() == 0) {
                    if (plan.isEmpty()) plan.add(new ArrayList<>());
                    plan.get(0).add(idx);
                    seen.add(idx);
                    map.put(proc.getName(), new Proc(idx));
                } else {
                    map.put(proc.getName(), new Proc(idx, cfg.deps.values()));
                    again = true;
                }
                idx++;
            }
            for (var entry : map.entrySet()) {
                if (entry.getValue().deps == null) continue;
                entry.getValue().resDeps = entry.getValue().deps.stream()
                    .map(d -> {
                        Proc p = map.get(d);
                        if (p == null) {
                            System.err.println("Referred to missing processor \"" + d + "\"");
                            return null;
                        } else return p.index;
                    })
                    .filter(i -> i != null && !seen.contains(i))
                    .collect(Collectors.toList());
            }

            if (plan.isEmpty() && again) {
                System.err.println("Cycle in processor dependencies");
            }

            for (int i = 1; again; i++) {
                again = false;
                for (Proc proc : map.values()) {
                    if (proc.resDeps != null && proc.resDeps.isEmpty()) {
                        again = true;
                        if (plan.size() == i) plan.add(new ArrayList<>());
                        plan.get(i).add(proc.index);
                        proc.resDeps = null;
                        for (Proc p2 : map.values()) if (p2.resDeps != null) p2.resDeps.remove(proc.index);
                    }
                }
            }

            for (Proc proc : map.values()) if (proc.resDeps != null) {
                System.err.println("Cycle in processor dependencies");
            }
        }
    }
    Executor exec;
    List<VisionProcessor> procs;
    NetworkTable table;
    BiConsumer<Mat, ? super CameraBase> postProcess;
    ConcurrentHashMap<CameraBase, CamState> states;
    boolean visionDebug;

    public VisionLibsGroup(List<VisionProcessor> procs, NetworkTable table, boolean visionDebug, Executor exec) {
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
        state.frames.add(frame);
        scheduleSelf(cam, state);
    }
    public Stream<VisionProcessor> getLibs(Collection<String> vlibs) {
        return procs.stream().filter(proc -> vlibs == null || vlibs.contains(proc.getName()));
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
        Mat frame = state.frames.poll();
        if (frame == null) {
            state.running.decrementAndGet();
            return;
        }
        state.buildPlan(cam.getConfig().vlibs);
        CompletableFuture<Void> future = CompletableFuture.completedFuture(null);
        for (int i = 0; i < state.plan.size(); i++) {
            int j = i; // make java happy about final variables
            future = future.thenCompose(_void -> {
                return CompletableFuture.allOf(
                    state.plan.get(j).stream()
                        .map(idx -> CompletableFuture.runAsync(() -> {
                            Map<String, VisionProcessor> deps = null;
                            VisionProcessor proc = procs.get(idx);
                            var cfg = proc.getConfig();
                            if (cfg != null) {
                                if (cfg.deps != null) {
                                    class Pair {
                                        String key;
                                        Optional<VisionProcessor> value;

                                        Pair(String key, Optional<VisionProcessor> value) {
                                            this.key = key;
                                            this.value = value;
                                        }
                                    }
                                    deps = cfg.deps.entrySet().stream()
                                        .map(e -> new Pair(
                                            e.getKey(),
                                            getLibs(cam.getConfig().vlibs)
                                                .filter(p -> p.getName().equals(e.getValue()))
                                                .findAny()
                                        ))
                                        .filter(p -> p.value.isPresent())
                                        .collect(Collectors.toMap(p -> p.key, p -> p.value.get()));
                                }
                            }
                            proc.process(frame, cam, deps);
                        }, exec))
                        .toArray(size -> new CompletableFuture[size])
                );
            });
        }
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
            .thenRunAsync(() -> {
                if (postProcess != null) postProcess.accept(frame, cam);
            }, exec)
            .exceptionally(e -> {
                e.printStackTrace(cam.getLog());
                return null;
            });
        cleanup.handle = fut;
        state.handles.put(fut, 0);
        fut.whenCompleteAsync(cleanup, exec);
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
        public synchronized void accept(Object _void, Object _ex) {
            if (handle != null) {
                state.handles.remove(handle);
            } else {
                cam.getLog().write("NULL handle, nothing to cleanup");
                cam.getLog().flush();
            }
            state.running.decrementAndGet();
            scheduleSelf(cam, state);
        }
    }
}
