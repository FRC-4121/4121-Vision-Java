package frc.vision.process;

import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraBase;
import java.util.Collection;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

// Most vision processors find some kind of bounding rectangle around their objects.
public abstract class ObjectVisionProcessor extends InstancedVisionProcessor<Collection<VisionObject>> {
    public Scalar rectColor;
    protected boolean calcAngles;

    protected ObjectVisionProcessor(String name) {
        this(name, new Scalar(1.0, 0.0, 0.0));
    }
    protected ObjectVisionProcessor(String name, Scalar color) {
        super(name);
        rectColor = color;
    }

    // Process the input image into a list of objects
    protected abstract Collection<VisionObject> processObjects(Mat img, CameraBase cam);
    
    @Override
    public void processStateful(Mat img, CameraBase cam, Ref state) {
        state.inner = processObjects(img, cam);
        if (calcAngles) state.inner.forEach(obj -> obj.calcAngles(cam.getConfig()));
    }

    @Override
    public void toNetworkTableStateful(NetworkTable table, Ref state) {
        NetworkTable table_ = table.getSubTable(name);
        int i = 0;
        int size = state.inner.size();
        double[] a = new double[size];
        double[] e = new double[size];
        double[] d = new double[size];
        for (VisionObject obj : state.inner) {
            a[i] = obj.azimuth;
            e[i] = obj.elevation;
            d[i] = obj.distance;
            ++i;
        }
        table_.putValue("a", NetworkTableValue.makeDoubleArray(a));
        table_.putValue("e", NetworkTableValue.makeDoubleArray(e));
        table_.putValue("d", NetworkTableValue.makeDoubleArray(d));
        table_.putValue("len", NetworkTableValue.makeInteger(size));
    }

    @Override
    public void drawOnImageStateful(Mat img, Ref state) {
        for (VisionObject obj : state.inner) {
            Imgproc.rectangle(img, obj.tl(), obj.br(), rectColor, 2);
            if (obj.hasAngles) {
                double x = obj.br().x;
                double y = obj.tl().y;
                Imgproc.putText(img, String.format("d: %.2f", obj.distance), new Point(x + 5, y + 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
                Imgproc.putText(img, String.format("a: %.2f", obj.azimuth), new Point(x + 5, y + 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
                Imgproc.putText(img, String.format("e: %.2f", obj.elevation), new Point(x + 5, y + 50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
            }
        }
    }
}
