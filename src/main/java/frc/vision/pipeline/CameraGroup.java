package frc.vision.pipeline;

import frc.vision.camera.*;
import frc.vision.load.CameraLoader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.stream.Stream;
import org.opencv.core.Mat;

public class CameraGroup {
    protected ArrayList<AsyncCameraThread> cams;
    protected boolean finished;

    public CameraGroup() {
        cams = new ArrayList();
    }

    // Run the finalization for all cameras.
    public void finish() {
        for (AsyncCameraThread cam : cams) {
            cam.getCamera().postInit();
        }
        finished = true;
    }

    // Load a camera with a given name and add it.
    public void add(String name) throws IllegalStateException, IOException {
        if (finished) throw new IllegalStateException("Cannot add another camera after finalization!");
        cams.add(new AsyncCameraThread(CameraLoader.load(name)));
    }

    // Add a camera to the group.
    public void add(CameraBase cam) throws IllegalStateException {
        if (finished) throw new IllegalStateException("Cannot add another camera after finalization!");
        cams.add(new AsyncCameraThread(cam));
    }

    // Load the given cameras by name, then finalize them.
    public static CameraGroup of(String... names) throws IOException {
        CameraGroup out = new CameraGroup();
        for (String name : names) {
            CameraBase cam = CameraLoader.load(name);
            out.cams.add(new AsyncCameraThread(cam));
        }
        out.finish();
        return out;
    }
    
    // Load the given cameras by name, then finalize them.
    public static CameraGroup of(Iterable<String> names) throws IOException {
        CameraGroup out = new CameraGroup();
        for (String name : names) {
            CameraBase cam = CameraLoader.load(name);
            out.cams.add(new AsyncCameraThread(cam));
        }
        out.finish();
        return out;
    }

    public int size() {
        return cams.size();
    }

    public boolean isFinished() {
        return finished;
    }

    public boolean isAlive() {
        for (AsyncCameraThread cam : cams) {
            if (cam.isAlive()) return true;
        }
        return false;
    }

    public void setCallback(BiConsumer<Mat, CameraBase> callback) {
        for (AsyncCameraThread cam : cams) cam.setCallback(callback);
    }

    // Start all of the camera threads.
    public void start() {
        if (!finished) finish();
        for (AsyncCameraThread cam : cams) cam.start();
    }

    // Politely request that the cameras all stop.
    public void cancel() {
        for (AsyncCameraThread cam : cams) cam.cancel();
    }

    // Forcibly stop all camera threads.
    public void stop() {
        for (AsyncCameraThread cam : cams) cam.stop();
    }

    // Flush all of the cameras' logs.
    public void flushLogs() {
        for (AsyncCameraThread cam : cams) cam.getCamera().getLog().flush();
    }

    // Get the cameras, as a stream so they can't be modified (too much)
    public Stream<AsyncCameraThread> getCams() {
        return cams.stream();
    }
}
