import java.io.FileNotFoundException;
import java.util.Calendar;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

class FrameCamera extends CameraBase {
    FrameCamera(String name, Mat img, Calendar date) throws FileNotFoundException {
        super(name, date);
        frame = new Mat();
        Imgproc.resize(img, frame, new Size(config.width, config.height));
    }
    FrameCamera(String name, Mat img) throws FileNotFoundException {
        super(name);
        frame = new Mat();
        Imgproc.resize(img, frame, new Size(config.width, config.height));
    }
    FrameCamera(String name, int type, Calendar date) throws FileNotFoundException {
        super(name, date);
        frame = new Mat(new Size(config.height, config.height), type, new Scalar(0));
    }
    FrameCamera(String name, int type) throws FileNotFoundException {
        super(name);
        frame = new Mat(new Size(config.width, config.height), type, new Scalar(0));
    }
    protected Mat readFrameRaw() {
        return frame;
    }
}
