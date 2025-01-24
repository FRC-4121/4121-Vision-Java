package frc.vision.pipeline;

import edu.wpi.first.networktables.NetworkTable;
import frc.vision.camera.*;
import frc.vision.process.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import java.util.function.BiConsumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.opencv.core.Mat;

// Vision library group to handle dispatch from a frame to running vision processors.
// Should be mostly non-blocking.
public class VisionLibsGroup implements BiConsumer<Mat, CameraBase> {
    protected class CamState {
        CompletableFuture<Void> handle;
        ArrayList<ArrayList<Integer>> plan;
        Mat frame;
        boolean loggedLibs;

        public CamState() {
            handle = CompletableFuture.completedFuture(null);
            frame = new Mat();
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
    CameraBase lastHandle;
    List<VisionProcessor> procs;
    NetworkTable table;
    BiConsumer<Mat, ? super CameraBase> postProcess;
    ConcurrentHashMap<CameraBase, CamState> states;
    boolean visionDebug;
    boolean cloneFrame;

    public VisionLibsGroup(List<VisionProcessor> procs, NetworkTable table, boolean visionDebug, Executor exec) {
        this.procs = procs;
        this.exec = exec;
        this.table = table;
        this.visionDebug = visionDebug;
        this.cloneFrame = visionDebug;

        states = new ConcurrentHashMap<CameraBase, CamState>();
    }

    public void setPostProcess(BiConsumer<Mat, ? super CameraBase> callback) {
        postProcess = callback;
    }

    /// Make sure we have our own copy of the frame. Might not be necessary?
    public void setCloneFrame(boolean cloneFrame) {
        this.cloneFrame = cloneFrame;
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
        if (frame == null) return;
        if (frame.dataAddr() == 0) return;
        CamState state = getState(cam);
        synchronized(state) {
            if (state.handle.isDone()) {
                state.frame = frame;
                scheduleSelf(cam, state);
            }
            // else state.ready.set(true);
        }
    }
    public Stream<VisionProcessor> getLibs(Collection<String> vlibs) {
        return procs.stream().filter(proc -> vlibs == null || vlibs.size() == 0 || vlibs.contains(proc.getName()));
    }
    public void add(VisionProcessor proc) {
        procs.add(proc);
    }
    protected void scheduleSelf(CameraBase cam, CamState state) {
        synchronized(state) {
            if (!state.loggedLibs) {
                String names = getLibs(cam.getConfig().vlibs)
                    .map(p -> p.getName())
                    .reduce((a, b) -> a + ", " + b)
                    .orElse("<none>");
                cam.getLog().write(String.format("Using processors: %s\n", names));
                cam.getLog().flush();
                state.loggedLibs = true;
            }
            state.buildPlan(cam.getConfig().vlibs);
            Mat frame = cloneFrame ? state.frame.clone() : state.frame;
            CompletableFuture<Void> future = CompletableFuture.completedFuture(null);
            for (int i = 0; i < state.plan.size(); i++) {
                int j = i; // make java happy about final variables
                future = future.thenCompose(_void -> {
                    return CompletableFuture.allOf(
                        state.plan.get(j).stream()
                            .map(idx -> CompletableFuture.runAsync(() -> procs.get(idx).process(frame, cam, null), exec))
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
            CompletableFuture fut = future
                .thenRunAsync(() -> postProcess.accept(frame, cam), exec)
                .exceptionally(e -> {
                    e.printStackTrace(cam.getLog());
                    return null;
                });
            state.handle = fut;
        }
    }
    public void cancel() {
        for (CamState state : states.values()) {
            state.handle.cancel(false);
        }
    }
    public boolean isRunning() {
        for (CamState state : states.values()) {
            if (!state.handle.isDone()) return true;
        }
        return false;
    }
}
