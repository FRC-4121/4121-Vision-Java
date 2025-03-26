package frc.vision.process;

import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraBase;
import java.util.Collection;
import java.util.Map;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

// Most vision processors find some kind of bounding rectangle around their objects.
public abstract class ObjectVisionProcessor<A> extends InstancedVisionProcessor<ObjectVisionProcessor<A>.Storage> {
    protected class Storage {
        Collection<VisionObject> seen;
        A additional;
    }

    public Scalar rectColor;
    protected boolean calcAngles;

    // Sentinel exception for processors that need state
    public static class NeedsState extends RuntimeException {}

    protected ObjectVisionProcessor(String name, ProcessorConfig cfg) {
        this(name, cfg, new Scalar(1.0, 0.0, 0.0));
    }
    protected ObjectVisionProcessor(String name, ProcessorConfig cfg, Scalar color) {
        super(name, cfg);
        rectColor = color;
    }

    // Process the input image into a list of objects, or return null if we need more state
    protected abstract Collection<VisionObject> processObjects(Mat img, CameraBase cfg, Map<String, VisionProcessor> deps);
    // Process the input image into a list of objects, with additional state
    protected Collection<VisionObject> processObjects(Mat img, CameraBase cfg, Map<String, VisionProcessor> deps, Ref<A> state) {
        return processObjects(img, cfg, deps);
    }

    @Override
    public void processStateful(Mat img, CameraBase cfg, Map<String, VisionProcessor> deps, Ref<Storage> state) {
        if (state.inner == null) state.inner = new Storage();
        state.inner.seen = processObjects(img, cfg, deps);
        if (state.inner.seen == null) {
            var r = new Ref<>(state.inner.additional);
            state.inner.seen = processObjects(img, cfg, deps, r);
            state.inner.additional = r.inner;
        }
        if (calcAngles) state.inner.seen.forEach(obj -> obj.calcAngles(cfg.getConfig()));
    }

    @Override
    public void toNetworkTableStateful(NetworkTable table, Ref<Storage> state) {
        NetworkTable table_ = table.getSubTable(name);
        int i = 0;
        int size = state.inner.seen.size();
        double[] a = new double[size];
        double[] e = new double[size];
        double[] d = new double[size];
        double[] r = new double[size];
        double[] o = new double[size];
        for (VisionObject obj : state.inner.seen) {
            a[i] = obj.azimuth;
            e[i] = obj.elevation;
            d[i] = obj.distance;
            r[i] = obj.rotation;
            o[i] = obj.offset;
            ++i;
        }
        table_.putValue("a", NetworkTableValue.makeDoubleArray(a));
        table_.putValue("e", NetworkTableValue.makeDoubleArray(e));
        table_.putValue("d", NetworkTableValue.makeDoubleArray(d));
        table_.putValue("r", NetworkTableValue.makeDoubleArray(r));
        table_.putValue("o", NetworkTableValue.makeDoubleArray(o));

        table_.putValue("len", NetworkTableValue.makeInteger(size));
    }

    @Override
    public void drawOnImageStateful(Mat img, Ref<Storage> state) {
        for (VisionObject obj : state.inner.seen) {
            Imgproc.rectangle(img, obj.tl(), obj.br(), rectColor, 2);
            if (obj.hasAngles) {
                double x = obj.br().x;
                double y = obj.tl().y;
                Imgproc.putText(img, String.format("d: %.2f", obj.distance), new Point(x + 5, y + 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
                Imgproc.putText(img, String.format("a: %.2f", obj.azimuth), new Point(x + 5, y + 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
                Imgproc.putText(img, String.format("e: %.2f", obj.elevation), new Point(x + 5, y + 50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
                Imgproc.putText(img, String.format("r: %.2f", obj.rotation), new Point(x + 5, y + 70), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
                Imgproc.putText(img, String.format("o: %.2f", obj.offset), new Point(x + 5, y + 90), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
            }
        }
    }
}
