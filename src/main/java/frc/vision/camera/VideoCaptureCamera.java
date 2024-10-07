package frc.vision.camera;

import frc.vision.load.*;
import java.io.IOException;
import java.time.LocalDateTime;
import org.opencv.core.*;
import org.opencv.videoio.*;

public class VideoCaptureCamera extends CameraBase {
    Mat currentFrame;
    VideoCapture cap;

    public VideoCaptureCamera(String name, VideoCapture cap, CameraConfig cfg, LocalDateTime date) throws IOException {
        super(name, cfg, date);
        this.currentFrame = new Mat();
        this.cap = cap;
    }
    public VideoCaptureCamera(String name, Config cfg, LocalDateTime date) throws IOException {
        super(name, cfg, date);
        currentFrame = new Mat();
        if (cfg.index != null) {
            cap = new VideoCapture(cfg.index);
        } else if (cfg.path != null) {
            cap = new VideoCapture(cfg.path);
        } else if (cfg.port != null) {
            Integer index = indexFromPort();
            if (index == null) {
                log.write(String.format("Couldn't find a camera on port %d\n", cfg.port));
            }
        } else {
            log.write("No way to select a camera was given!\n");
        }
        if (cap != null) {
            try {
                if (cfg.width != 0) {
                    cap.set(Videoio.CAP_PROP_FRAME_WIDTH, cfg.width);
                }
                if (cfg.height != 0) {
                    cap.set(Videoio.CAP_PROP_FRAME_WIDTH, cfg.width);
                }
                if (cfg.fps != 0) {
                    cap.set(Videoio.CAP_PROP_FPS, cfg.fps);
                }
                if (cfg.brightness != null) {
                    cap.set(Videoio.CAP_PROP_BRIGHTNESS, cfg.brightness);
                }
                if (cfg.fourcc != null) {
                    if (cfg.fourcc.length() == 4) {
                        cap.set(Videoio.CAP_PROP_FOURCC,
                            VideoWriter.fourcc(
                                cfg.fourcc.charAt(0),
                                cfg.fourcc.charAt(1),
                                cfg.fourcc.charAt(2),
                                cfg.fourcc.charAt(3)
                            )
                        );
                    } else {
                        log.write(String.format("Invalid fourcc \"%s\"\n", cfg.fourcc));
                    }
                }
            } catch (Exception e) {
                e.printStackTrace(log);
            }
        }
        log.flush();
    }

    public VideoCaptureCamera(String name, VideoCapture cap, CameraConfig cfg) throws IOException {
        this(name, cap, cfg, LocalDateTime.now());
    }
    public VideoCaptureCamera(String name, Config cfg) throws IOException {
        this(name, cfg, LocalDateTime.now());
    }

    public static Integer indexFromPort() {
        try {
            return null; // TODO
        } catch (Exception e) {
            return null;
        }
    }
    
    @Override
    public Mat readFrameRaw() {
        if (cap == null) return null;
        boolean ok = cap.read(currentFrame);
        return ok ? currentFrame : null;
    }

    public static class Config extends CameraConfig {
        Integer index;
        String path;
        Integer port;

        double fps;
        Double brightness;

        String fourcc;
    }

    public static class Factory extends CameraFactory {
        @Override
        public Class<Config> configType() {
            return Config.class;
        }

        @Override
        public String typeName() {
            return "cv";
        }

        @Override
        public VideoCaptureCamera create(String name, CameraConfig cfg, LocalDateTime date) throws IOException {
            return new VideoCaptureCamera(name, (Config)cfg, date);
        }
    }
}
