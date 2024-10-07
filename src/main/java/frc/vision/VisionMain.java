package frc.vision;

import frc.vision.camera.*;
import frc.vision.load.*;
import frc.vision.pipeline.*;
import frc.vision.process.*;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.Set;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;

public class VisionMain {
    protected static final String logNameFormat = "log_%s_%d.txt";

    public static void main(String[] args) throws Exception {
        boolean visionDebug = true;
        File logDir = new File("logs");
        
        long pid = ProcessHandle.current().pid();
        System.out.println("PID: " + pid);
        LocalDateTime time = LocalDateTime.now();
        System.out.println("running at " + String.valueOf(time));

        CameraBase.logDir = new File(logDir, "cam");
        File runLogs = new File(logDir, "run");
        File runLog = new File(runLogs, String.format(logNameFormat, CameraBase.logDateFormat.format(time), pid));
        final PrintWriter log = new PrintWriter(runLog);
        
        CameraGroup cams = null;

        try {
            log.write(String.format("Running with PID %d at %s\n", pid, time));
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

            CameraLoader.registerFactory(new FrameCamera.Factory());
            CameraLoader.registerFactory(new VideoCaptureCamera.Factory());
            CameraLoader.initConfig();

            ProcessorLoader.registerFactory(new FpsCounter.Factory());
            ProcessorLoader.registerFactory(new AprilTagProcessor.Factory());
            ProcessorLoader.registerFactory(new RectVisionProcessor.Factory());
            ProcessorLoader.initConfig();

            Executor exec = ForkJoinPool.commonPool();

            VisionLibsGroup procs = new VisionLibsGroup(
                ProcessorLoader.loadAll("fps", "april", "ring2024"),
                null, visionDebug, exec
            );

            {
                String names = procs.getLibs(null)
                    .map(p -> p.getName())
                    .reduce((a, b) -> a + ", " + b)
                    .orElse("<none>");
                log.write(String.format("Loaded processors: %s\n", names));
            }
            

            ImShower imgs = new ImShower();
            if (visionDebug) {
               procs.setPostProcess(imgs);
            }

            cams = CameraGroup.of("idx0", "dummy");
            cams.setCallback(procs);

            {
                String names = cams.getCams()
                    .map(p -> p.getCamera().getName())
                    .reduce((a, b) -> a + ", " + b)
                    .orElse("<none>");
                log.write(String.format("Loaded cameras: %s\n", names));
            }
            
            log.flush();

            cams.start();


            System.out.println("Running successfully :3");
            
            while (true) {
                if (visionDebug) {
                    imgs.run();
                    if (imgs.canWaitKey()) {
                        int res = HighGui.waitKey(10);
                        if (res == 27) {
                            log.write("ESC key pressed\n");
                            break;
                        }
                    }
                }
            }

            cams.cancel();

            if (visionDebug) HighGui.waitKey(1); // appease opencv
        } catch (Exception e) {
            e.printStackTrace(log);
        } finally {
            if (cams != null) cams.flushLogs();
            log.write(String.format("Run ended at %s\n", String.valueOf(LocalDateTime.now())));
            log.flush();
        }
    }
}
