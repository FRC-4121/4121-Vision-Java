package frc.vision.camera;

import frc.vision.load.*;
import java.io.IOException;
import java.time.LocalDateTime;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class FrameCamera extends CameraBase {
    protected Mat thisFrame;
    public FrameCamera(String name, Mat img, Config cfg, LocalDateTime date) throws IOException {
        super(name, cfg, date);
        Imgproc.resize(img, thisFrame, new Size(config.width, config.height));
    }
    public FrameCamera(String name, Mat img, Config cfg) throws IOException {
        this(name, img, cfg, LocalDateTime.now());
    }
    public FrameCamera(String name, Config cfg, LocalDateTime date) throws IOException {
        super(name, cfg, date);
        Size size = new Size(config.height, config.height);
        try {
            if (cfg.filePath != null) {
                Mat img = Imgcodecs.imread(cfg.filePath);
                Imgproc.resize(img, thisFrame, size);
            }
        }
        finally {
            if (thisFrame == null) {
                thisFrame = new Mat(size, CvType.CV_8UC3, new Scalar(0));
            }
        }
    }
    public FrameCamera(String name, Config cfg) throws IOException {
        this(name, cfg, LocalDateTime.now());
    }

    @Override
    protected Mat readFrameRaw() {
        if (frame == null) {
            return thisFrame.clone();
        }
        thisFrame.copyTo(frame);
        return frame;
    }

    public static class Config extends CameraConfig {
        String filePath;
    }

    public static class Factory extends CameraFactory {
        @Override
        public Class<Config> configType() {
            return Config.class;
        }

        @Override
        public String typeName() {
            return "frame";
        }

        @Override
        public FrameCamera create(String name, CameraConfig cfg, LocalDateTime date) throws IOException {
            return new FrameCamera(name, (Config)cfg, date);
        }
    }
}
