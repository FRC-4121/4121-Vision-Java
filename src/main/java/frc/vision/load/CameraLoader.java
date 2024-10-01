package frc.vision.load;

import com.google.gson.*;
import com.google.gson.reflect.TypeToken;
import frc.vision.camera.CameraBase;
import frc.vision.camera.CameraConfig;
import java.lang.Class;
import java.lang.reflect.Type;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.Reader;
import java.time.LocalDateTime;
import java.util.HashMap;
import java.util.Map;

// A camera that can be loaded from the configuration file.
public class CameraLoader {
    protected static HashMap<String, CameraFactory> types = new HashMap();
    protected static HashMap<String, WrappedConfig> configs = new HashMap();
    private static boolean configInitialized = false;

    protected static class WrappedConfig {
        CameraConfig inner;
        public WrappedConfig(CameraConfig inner) {
            this.inner = inner;
        }
    }

    protected static class CustomDeserializer implements JsonDeserializer<WrappedConfig> {
        @Override
        public WrappedConfig deserialize(JsonElement json, Type type, JsonDeserializationContext context) throws JsonParseException {
            JsonObject obj = json.getAsJsonObject();
            String ty = obj.get("type").getAsString();
            CameraFactory factory = types.get(ty);
            CameraConfig cfg = context.deserialize(json, factory.configType());
            return new WrappedConfig(cfg);
        }
    }

    protected CameraLoader() {}

    // Initialize configs from a file, in JSON.
    public static void initConfig(Reader file) throws JsonSyntaxException, JsonIOException {
        Gson gson = new GsonBuilder()
            .registerTypeAdapter(WrappedConfig.class, new CustomDeserializer())
            .create();
        configs = gson.fromJson(file, new TypeToken<HashMap<String, WrappedConfig>>() {}.getType());
        
        for (Map.Entry<String, WrappedConfig> entry : configs.entrySet()) {
            if (entry.getKey() == "default") continue;
            WrappedConfig default_ = configs.get("default");
            if (default_ != null) entry.getValue().inner.updateFrom(default_.inner);
        }
        configInitialized = true;
    }
    public static void initConfig() throws JsonSyntaxException, JsonIOException, FileNotFoundException {
        var res = CameraLoader.class
            .getClassLoader()
            .getResource("cameras.json");
        if (res == null) return;
        initConfig(new FileReader(res.getFile()));
    }

    // Register a factory to create cameras.
    public static void registerFactory(CameraFactory factory) {
        types.put(factory.typeName(), factory);
    }

    // Load a camera with the given name.
    // TODO: give better exceptions.
    public static CameraBase load(String name, LocalDateTime date) throws FileNotFoundException {
        WrappedConfig wcfg = configs.get(name);
        CameraConfig cfg = wcfg.inner;
        CameraFactory factory = types.get(cfg.type);
        return factory.create(name, cfg, date);
    }
    // Load a camera with the given name.
    // TODO: give better exceptions.
    public static CameraBase load(String name) throws FileNotFoundException {
        return load(name, LocalDateTime.now());
    }
}
