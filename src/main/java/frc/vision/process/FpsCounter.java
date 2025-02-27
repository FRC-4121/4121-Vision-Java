package frc.vision.process;

import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraBase;
import frc.vision.load.*;
import java.time.*;
import java.util.Map;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class FpsCounter extends InstancedVisionProcessor<FpsCounter.State> {
    static class State {
        Instant last;
        double lastFps;
        double minFps;
        double maxFps;
        double avgFps;
        int numFrames;
    }
    public FpsCounter() {
        this("FPS");
    }
    public FpsCounter(String name) {
        super(name, null);
    }

    @Override
    protected void processStateful(Mat _img, CameraBase _cfg, Map<String, VisionProcessor> _deps, Ref state) {
        if (state.inner == null) {
            state.inner = new State();
            state.inner.minFps = Float.POSITIVE_INFINITY;
            state.inner.maxFps = Float.NEGATIVE_INFINITY;
        }
        Instant inst = Instant.now();
        State s = state.inner;
        if (s.last != null) {
            Duration diff = Duration.between(s.last, inst);
            double lastFps = 1000000000 / (double)diff.toNanos();
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
        if (state.inner == null) return;
        State s = state.inner;

        table.putValue("fps", NetworkTableValue.makeDouble(s.lastFps));
        table.putValue("minFps", NetworkTableValue.makeDouble(s.minFps));
        table.putValue("maxFps", NetworkTableValue.makeDouble(s.maxFps));
        table.putValue("avgFps", NetworkTableValue.makeDouble(s.avgFps));
    }

    @Override
    protected void drawOnImageStateful(Mat img, Ref state) {
        if (state.inner == null) return;
        State s = state.inner;
        String text = String.format("%5.1f/%5.1f/%5.1f/%5.1f", s.lastFps, s.minFps, s.maxFps, s.avgFps);
        Imgproc.putText(img, text, new Point(10, 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255));
    }

    public static class Factory extends ProcessorFactory {
        @Override
        public String typeName() {
            return "fps";
        }
        @Override
        public FpsCounter create(String name, ProcessorConfig cfg) {
            return new FpsCounter(name);
        }
    }
}
