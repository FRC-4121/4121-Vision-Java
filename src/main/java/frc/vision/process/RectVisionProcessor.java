package frc.vision.process;

import frc.vision.camera.CameraConfig;
import frc.vision.load.ProcessorFactory;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class RectVisionProcessor extends ObjectVisionProcessor {
    protected Config cfg;

    public RectVisionProcessor(String name, Config cfg) {
        super(name, cfg.rectColor());
        this.cfg = cfg;
    }
    
    protected Collection<VisionObject> processObjects(Mat img, CameraConfig ccfg) {
        Mat raw = img.clone();
        Mat mat2 = new Mat();
        Imgproc.GaussianBlur(raw, mat2, new Size(13, 13), 0);
        Mat hsv = new Mat();
        Imgproc.cvtColor(mat2, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(
            hsv,
            new Scalar(cfg.hmin, cfg.smin, cfg.vmin),
            new Scalar(cfg.hmax, cfg.smax, cfg.vmax),
            mat2
        );
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mat2, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Imgproc.findContoursLinkRuns(mat2, contours); // exists in docs but not code?
        contours.sort(Comparator.comparing(c -> Integer.MAX_VALUE - c.size().area()));
        ArrayList<VisionObject> out = new ArrayList<VisionObject>();
        double expAspect = cfg.height / cfg.width;
        double altAspect = cfg.sideways ? 1 / expAspect : expAspect;
        for (Mat c : contours) {
            VisionObject obj = new VisionObject(Imgproc.boundingRect(c));
            if (obj.area() < cfg.minArea) break;
            if (cfg.maxArea != 0 && obj.area() > cfg.maxArea) continue;
            double aspect = (double)obj.height / obj.width;
            if (
                ((aspect < expAspect * (1 - cfg.tolerance)) || // too wide for main
                (aspect > expAspect * (1 + cfg.tolerance))) && // too tall for main
                ((aspect < altAspect * (1 - cfg.tolerance)) || // too wide for alt
                (aspect > altAspect * (1 + cfg.tolerance)))    // too tall for alt
            ) continue;
            out.add(obj);
        }
        return out;
    }

    public static class Config extends ProcessorConfig {
        int hmin = 0;
        int hmax = 255;
        int smin = 0;
        int smax = 255;
        int vmin = 0;
        int vmax = 255;

        double width;
        double height;

        double tolerance = 100.0;
        boolean sideways = false;
        int minArea = 0;
        int maxArea = 0;

        public Scalar avgColor(boolean maybeInvert) {
            int h = (hmax + hmin) / 2;
            if (hmin > hmax) h += 128;
            h %= 256;
            int s = (smin + smax) / 2;
            if (maybeInvert && s < 64) s = 256 - s;
            int v = (vmin + vmax) / 2;
            Mat hsv = new Mat(1, 1, CvType.CV_8UC3, new Scalar(h, s, v));
            Mat bgr = new Mat();
            Imgproc.cvtColor(hsv, bgr, Imgproc.COLOR_HSV2BGR);
            byte[] px = new byte[3];
            bgr.get(0, 0, px);
            return new Scalar(
                Byte.toUnsignedInt(px[0]),
                Byte.toUnsignedInt(px[1]),
                Byte.toUnsignedInt(px[2])
            );
        }
        public Scalar rectColor() {
            Scalar avg = avgColor(true);
            return new Scalar(
                255 - avg.val[0],
                255 - avg.val[1],
                255 - avg.val[2]
            );
        }
    }

    public static class Factory extends ProcessorFactory {
        @Override
        public String typeName() {
            return "rect";
        }
        @Override
        public Class<Config> configType() {
            return Config.class;
        }
        @Override
        public RectVisionProcessor create(String name, ProcessorConfig cfg) {
            return new RectVisionProcessor(name, (Config)cfg);
        }
    }
}
