package first.robot.drivers;

import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Polls the local sc-witmotion service and caches the latest decoded IMU sample for robot code.
 */
public final class ScWitMotionDriver implements AutoCloseable {
  private static final Duration DEFAULT_POLL_PERIOD = Duration.ofMillis(20);
  private static final Duration DEFAULT_REQUEST_TIMEOUT = Duration.ofMillis(100);
  private static final String DEFAULT_ENDPOINT = "http://127.0.0.1:9010/api/v1/imu.kv";

  private final HttpClient httpClient;
  private final HttpRequest request;
  private final ScheduledExecutorService executor;
  private final Duration pollPeriod;
  private final AtomicReference<Sample> latestSample;
  private final AtomicReference<String> lastPollError;

  private volatile boolean started;

  public ScWitMotionDriver() {
    this(DEFAULT_ENDPOINT, DEFAULT_POLL_PERIOD, DEFAULT_REQUEST_TIMEOUT);
  }

  public ScWitMotionDriver(String endpoint) {
    this(endpoint, DEFAULT_POLL_PERIOD, DEFAULT_REQUEST_TIMEOUT);
  }

  public ScWitMotionDriver(String endpoint, Duration pollPeriod, Duration requestTimeout) {
    Objects.requireNonNull(endpoint, "endpoint");
    this.pollPeriod = Objects.requireNonNull(pollPeriod, "pollPeriod");
    Objects.requireNonNull(requestTimeout, "requestTimeout");
    this.httpClient = HttpClient.newBuilder().connectTimeout(requestTimeout).build();
    this.request = HttpRequest.newBuilder(URI.create(endpoint)).timeout(requestTimeout).GET().build();
    this.executor =
        Executors.newSingleThreadScheduledExecutor(
            runnable -> {
              Thread thread = new Thread(runnable, "sc-witmotion-driver");
              thread.setDaemon(true);
              return thread;
            });
    this.latestSample = new AtomicReference<>(Sample.disconnected());
    this.lastPollError = new AtomicReference<>("");
  }

  public synchronized void start() {
    if (started) {
      return;
    }
    started = true;
    executor.scheduleWithFixedDelay(this::pollSafely, 0L, pollPeriod.toMillis(), TimeUnit.MILLISECONDS);
  }

  public void pollOnce() throws IOException, InterruptedException {
    HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
    if (response.statusCode() != 200) {
      throw new IOException("HTTP " + response.statusCode());
    }
    latestSample.set(parseSample(response.body()));
    lastPollError.set("");
  }

  public Sample getLatestSample() {
    return latestSample.get();
  }

  public String getLastPollError() {
    return lastPollError.get();
  }

  public boolean isConnected() {
    return latestSample.get().isConnected();
  }

  public double getRollDegrees() {
    return latestSample.get().getRollDegrees();
  }

  public double getPitchDegrees() {
    return latestSample.get().getPitchDegrees();
  }

  public double getYawDegrees() {
    return latestSample.get().getYawDegrees();
  }

  @Override
  public synchronized void close() {
    executor.shutdownNow();
    started = false;
  }

  private void pollSafely() {
    try {
      pollOnce();
    } catch (Exception ex) {
      lastPollError.set(ex.getMessage() == null ? ex.getClass().getSimpleName() : ex.getMessage());
    }
  }

  private static Sample parseSample(String body) {
    Map<String, String> values = new HashMap<>();
    for (String line : body.split("\\R")) {
      if (line.isBlank()) {
        continue;
      }
      int split = line.indexOf('=');
      if (split <= 0) {
        continue;
      }
      values.put(line.substring(0, split), line.substring(split + 1));
    }

    return new Sample(
        parseFlag(values.get("connected")),
        parseFlag(values.get("stale")),
        parseLong(values.get("timestamp_ms")),
        parseLong(values.get("age_ms")),
        parseLong(values.get("sequence")),
        parseDouble(values.get("sample_rate_hz")),
        parseDouble(values.get("roll_deg")),
        parseDouble(values.get("pitch_deg")),
        parseDouble(values.get("yaw_deg")),
        parseDouble(values.get("accel_x_g")),
        parseDouble(values.get("accel_y_g")),
        parseDouble(values.get("accel_z_g")),
        parseDouble(values.get("gyro_x_dps")),
        parseDouble(values.get("gyro_y_dps")),
        parseDouble(values.get("gyro_z_dps")),
        parseDouble(values.get("mag_x")),
        parseDouble(values.get("mag_y")),
        parseDouble(values.get("mag_z")),
        parseDouble(values.get("temperature_c")),
        parseDouble(values.get("version_raw")),
        values.getOrDefault("device", ""),
        parseLong(values.get("baud")),
        values.getOrDefault("last_error", ""));
  }

  private static boolean parseFlag(String value) {
    return "1".equals(value) || "true".equalsIgnoreCase(value);
  }

  private static long parseLong(String value) {
    if (value == null || value.isBlank()) {
      return 0L;
    }
    try {
      return Long.parseLong(value.trim());
    } catch (NumberFormatException ex) {
      return 0L;
    }
  }

  private static double parseDouble(String value) {
    if (value == null || value.isBlank()) {
      return Double.NaN;
    }
    try {
      return Double.parseDouble(value.trim());
    } catch (NumberFormatException ex) {
      return Double.NaN;
    }
  }

  public static final class Sample {
    private final boolean connected;
    private final boolean stale;
    private final long timestampMs;
    private final long ageMs;
    private final long sequence;
    private final double sampleRateHz;
    private final double rollDegrees;
    private final double pitchDegrees;
    private final double yawDegrees;
    private final double accelXG;
    private final double accelYG;
    private final double accelZG;
    private final double gyroXDps;
    private final double gyroYDps;
    private final double gyroZDps;
    private final double magX;
    private final double magY;
    private final double magZ;
    private final double temperatureC;
    private final double versionRaw;
    private final String device;
    private final long baud;
    private final String serviceError;

    public Sample(
        boolean connected,
        boolean stale,
        long timestampMs,
        long ageMs,
        long sequence,
        double sampleRateHz,
        double rollDegrees,
        double pitchDegrees,
        double yawDegrees,
        double accelXG,
        double accelYG,
        double accelZG,
        double gyroXDps,
        double gyroYDps,
        double gyroZDps,
        double magX,
        double magY,
        double magZ,
        double temperatureC,
        double versionRaw,
        String device,
        long baud,
        String serviceError) {
      this.connected = connected;
      this.stale = stale;
      this.timestampMs = timestampMs;
      this.ageMs = ageMs;
      this.sequence = sequence;
      this.sampleRateHz = sampleRateHz;
      this.rollDegrees = rollDegrees;
      this.pitchDegrees = pitchDegrees;
      this.yawDegrees = yawDegrees;
      this.accelXG = accelXG;
      this.accelYG = accelYG;
      this.accelZG = accelZG;
      this.gyroXDps = gyroXDps;
      this.gyroYDps = gyroYDps;
      this.gyroZDps = gyroZDps;
      this.magX = magX;
      this.magY = magY;
      this.magZ = magZ;
      this.temperatureC = temperatureC;
      this.versionRaw = versionRaw;
      this.device = device;
      this.baud = baud;
      this.serviceError = serviceError;
    }

    public static Sample disconnected() {
      return new Sample(
          false,
          true,
          0L,
          Long.MAX_VALUE,
          0L,
          0.0,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          "",
          0L,
          "");
    }

    public boolean isConnected() {
      return connected;
    }

    public boolean isStale() {
      return stale;
    }

    public boolean isFresh() {
      return connected && !stale;
    }

    public long getTimestampMs() {
      return timestampMs;
    }

    public long getAgeMs() {
      return ageMs;
    }

    public long getSequence() {
      return sequence;
    }

    public double getSampleRateHz() {
      return sampleRateHz;
    }

    public double getRollDegrees() {
      return rollDegrees;
    }

    public double getPitchDegrees() {
      return pitchDegrees;
    }

    public double getYawDegrees() {
      return yawDegrees;
    }

    public double getAccelXG() {
      return accelXG;
    }

    public double getAccelYG() {
      return accelYG;
    }

    public double getAccelZG() {
      return accelZG;
    }

    public double getGyroXDps() {
      return gyroXDps;
    }

    public double getGyroYDps() {
      return gyroYDps;
    }

    public double getGyroZDps() {
      return gyroZDps;
    }

    public double getMagX() {
      return magX;
    }

    public double getMagY() {
      return magY;
    }

    public double getMagZ() {
      return magZ;
    }

    public double getTemperatureC() {
      return temperatureC;
    }

    public double getVersionRaw() {
      return versionRaw;
    }

    public String getDevice() {
      return device;
    }

    public long getBaud() {
      return baud;
    }

    public String getServiceError() {
      return serviceError;
    }
  }
}
