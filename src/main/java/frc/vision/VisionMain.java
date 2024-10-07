package frc.vision;

import frc.vision.camera.*;
import frc.vision.load.*;
import frc.vision.pipeline.*;
import frc.vision.process.*;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.TreeSet;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;

public class VisionMain {
    private enum CliState {
        NORMAL,
        LOG_DIR,
    };

    protected static final String logNameFormat = "log_%s_%s_%d.txt";

    public static void main(String[] args) throws Exception {
        boolean visionDebug = false;
        TreeSet camNames = new TreeSet();
        File logDir = new File("logs");

        CliState state = CliState.NORMAL;
        for (String arg : args) {
            switch (state) {
                case NORMAL:
                    if (arg.charAt(0) == '-') {
                            if (arg.charAt(1) == '-') {
                            String longFlag = arg.substring(2);
                            if (longFlag == "vision-debug") {
                                visionDebug = true;
                            } else if (longFlag == "log-dir") {
                                state = CliState.LOG_DIR;
                            } else {
                                System.err.println(String.format("Unknown long flag \"%s\"", longFlag));
                                return;
                            }
                        } else {
                            for (char c : arg.substring(1).toCharArray()) {
                                switch (c) {
                                    case 'v':
                                        visionDebug = true;
                                        break;
                                    case 'l':
                                        if (state != CliState.NORMAL) {
                                            System.err.println("l flag expects the next argument to be the log directory, but another flag already is expecting something");
                                            return;
                                        }
                                        state = CliState.LOG_DIR;
                                        break;
                                    default:
                                        System.err.println(String.format("Unknown short flag \"%c\"", c));
                                }
                            }
                        }
                    } else {
                        if (camNames.contains(arg)) {
                            System.err.println(String.format("Duplicate camera \"%s\"", arg));
                        }
                        camNames.add(arg);
                    }
                    break;
                case LOG_DIR:
                    logDir = new File(arg);
                    state = CliState.NORMAL;
                    break;
            }
        }

        switch (state) {
            case NORMAL: break;
            case LOG_DIR:
                System.err.println("Expected a log directory but no more arguments were passed");
                return;
        }

        if (camNames.isEmpty()) {
            System.err.println("No cameras were specified!");
        }
 
        long pid = ProcessHandle.current().pid();
        System.out.println("PID: " + pid);
        LocalDateTime time = LocalDateTime.now();
        System.out.println("running at " + String.valueOf(time));

        CameraBase.logDir = new File(logDir, "cam");
        File runLogs = new File(logDir, "run");
        File runLog = new File(runLogs, String.format(logNameFormat, String.join("_", camNames), CameraBase.logDateFormat.format(time), pid));
        PrintWriter log = new PrintWriter(runLog);
        
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

            cams = CameraGroup.of("idx0");
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
