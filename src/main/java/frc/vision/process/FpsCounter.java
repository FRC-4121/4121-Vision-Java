package frc.vision.process;

import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraConfig;
import java.time.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class FpsCounter extends InstancedVisionProcessor<FpsCounter.State> {
    static class State {
        Instant last;
        float lastFps;
        float minFps;
        float maxFps;
        float avgFps;
        int numFrames;
    }
    public FpsCounter() {
        this("FPS");
    }
    public FpsCounter(String name) {
        super(name);
    }

    @Override
    protected void processStateful(Mat _img, CameraConfig _cfg, Ref state) {
        System.out.println("process");
        if (state.inner == null) {
            state.inner = new State();
            state.inner.minFps = Float.POSITIVE_INFINITY;
            state.inner.maxFps = Float.NEGATIVE_INFINITY;
        }
        Instant inst = Instant.now();
        State s = state.inner;
        if (s.last != null) {
            Duration diff = Duration.between(s.last, inst);
            float lastFps = 1000000000 / (float)diff.toNanos();
            s.lastFps = lastFps;
            if (lastFps < s.minFps) {
                s.minFps = lastFps;
            }
            if (lastFps > s.maxFps) {
                s.maxFps = lastFps;
            }
            s.avgFps *= s.numFrames;
            s.avgFps += lastFps;
            s.numFrames++;
            s.avgFps /= s.numFrames;
        }
        s.last = inst;
    }

    @Override
    protected void toNetworkTableStateful(NetworkTable table, Ref state) {
        System.out.println("table");
        if (state.inner == null) return;
        State s = state.inner;

        table.putValue("fps", NetworkTableValue.makeFloat(s.lastFps));
        table.putValue("minFps", NetworkTableValue.makeFloat(s.minFps));
        table.putValue("maxFps", NetworkTableValue.makeFloat(s.maxFps));
        table.putValue("avgFps", NetworkTableValue.makeFloat(s.avgFps));
    }

    @Override
    protected void drawOnImageStateful(Mat img, Ref state) {
        System.out.println("draw");
        if (state.inner == null) return;
        State s = state.inner;
 
        String text = String.format("%3.1d/%3.1d/%3.1d/%3.1d", s.lastFps, s.minFps, s.maxFps, s.avgFps);
        Imgproc.putText(img, text, new Point(10, 10), Imgproc.FONT_HERSHEY_PLAIN, 0.5, new Scalar(1, 1, 1));
    }
}
