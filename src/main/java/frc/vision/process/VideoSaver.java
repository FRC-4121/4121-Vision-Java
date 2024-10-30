package frc.vision.process;

import edu.wpi.first.networktables.NetworkTable;
import frc.vision.camera.CameraBase;
import frc.vision.load.*;
import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.Duration;
import java.time.Instant;
import java.time.LocalDateTime;
import java.util.concurrent.ConcurrentHashMap;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;

public class VideoSaver extends VisionProcessor {
    protected File savePath;
    protected double targetFps;
    protected int fourcc;
    protected String extension;
    protected ConcurrentHashMap<CameraBase, VideoWriter> cams;
    protected LocalDateTime startupTime;

    public Duration lostTime;
    public int framesWritten;

    public static File defaultSavePath = new File("videos");

    public VideoSaver(String name) {
        this(name, 30, null);
    }
    public VideoSaver(String name, double fps) {
        this(name, fps, null);
    }
    public VideoSaver(String name, File savePath) {
        this(name, 30, savePath);
    }
    public VideoSaver(String name, double fps, File savePath) {
        super(name);
        this.savePath = savePath;
        this.targetFps = fps;
        this.cams = new ConcurrentHashMap<CameraBase, VideoWriter>();
        this.fourcc = VideoWriter.fourcc('M', 'J', 'P', 'G');
        this.extension = "avi";
        this.startupTime = LocalDateTime.now();
        this.lostTime = Duration.ZERO;
    }

    public void register(CameraBase camera) {
        cams.computeIfAbsent(camera, cam -> {
            String filename = String.format("%s_%s.%s", cam.getName(), CameraBase.logDateFormat.format(startupTime), extension);
            File link = new File(savePath, String.format("%s_LATEST.%s", cam.getName(), extension));
            link.delete();
            try {
                Files.createSymbolicLink(link.toPath(), Paths.get(filename));
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
            return new VideoWriter(
                new File(
                    savePath,
                    filename
                ).getPath(),
                fourcc,
                targetFps,
                new Size(cam.getConfig().width, cam.getConfig().height)
            );
        });
    }

    @Override
    public void process(Mat img, CameraBase handle) {
        register(handle);
    }
    @Override
    public void toNetworkTable(NetworkTable table, CameraBase handle) {}
    @Override
    public void drawOnImage(Mat img, CameraBase handle) {
        Size sz = img.size();
        Imgproc.circle(img, new Point(sz.width - 15, 15), 10, new Scalar(0, 0, 255), -1);
    }

    public static class Config extends Typed {
        public double fps = 30;
        public String savePath = defaultSavePath.getPath();
    }
    public static class Factory extends ProcessorFactory {
        @Override
        public String typeName() {
            return "video";
        }
        @Override
        public Class<Config> configType() {
            return Config.class;
        }
        @Override
        public VideoSaver create(String name, Typed cfg) {
            Config config = (Config)cfg;
            VideoSaver out = new VideoSaver(name, config.fps, config.savePath != null ? new File(config.savePath) : null);
            out.new WriteThread().start();
            return out;
        }
    }
    public class WriteThread extends Thread {
        public boolean running = true;
        protected Instant lastTime;

        public WriteThread() {
            setName("VideoSaver-" + name);
            setDaemon(true);
        }

        @Override
        public void run() {
            while (running) {
                cams.forEach((cam, writer) -> writer.write(cam.getFrame()));
                framesWritten++;
                if (lastTime != null) {
                    long toSleep = (long)(1000 / targetFps - Duration.between(lastTime, Instant.now()).toMillis());
                    if (toSleep > 0) {
                        try {
                            sleep(toSleep);
                        } catch (Exception e) {}
                    } else {
                        lostTime = lostTime.plusMillis(-toSleep);
                    }
                }
                lastTime = Instant.now();
            }
        }
    }
}
