package frc.vision.process;

import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraConfig;
import java.util.Collection;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

// Most vision processors find some kind of bounding rectangle around their objects.
public abstract class ObjectVisionProcessor extends InstancedVisionProcessor<Collection<VisionObject>> {
    public Scalar rectColor;

    protected ObjectVisionProcessor(String name) {
        this(name, new Scalar(1.0, 0.0, 0.0));
    }
    protected ObjectVisionProcessor(String name, Scalar color) {
        super(name);
        rectColor = color;
    }

    // Process the input image into a list of objects
    protected abstract Collection<VisionObject> processObjects(Mat img, CameraConfig cfg);
    
    @Override
    public void processStateful(Mat img, CameraConfig cfg, Ref state) {
        state.inner = processObjects(img, cfg);
    }

    @Override
    public void toNetworkTableStateful(NetworkTable table, Ref state) {
        int i = 0;
        int size = state.inner.size();
        long[] x = new long[size];
        long[] y = new long[size];
        long[] w = new long[size];
        long[] h = new long[size];
        double[] o = new double[size];
        for (VisionObject obj : state.inner) {
            x[i] = obj.x;
            y[i] = obj.y;
            w[i] = obj.width;
            h[i] = obj.height;
            o[i] = obj.offset();
        }
        table.putValue("x", NetworkTableValue.makeIntegerArray(x));
        table.putValue("y", NetworkTableValue.makeIntegerArray(y));
        table.putValue("w", NetworkTableValue.makeIntegerArray(w));
        table.putValue("h", NetworkTableValue.makeIntegerArray(h));
        table.putValue("o", NetworkTableValue.makeDoubleArray(o));
        table.putValue("len", NetworkTableValue.makeInteger(size));
    }

    @Override
    public void drawOnImageStateful(Mat img, Ref state) {
        for (VisionObject obj : state.inner) {
            Imgproc.rectangle(img, obj.tl(), obj.br(), rectColor, 2);
        }
    }
}
