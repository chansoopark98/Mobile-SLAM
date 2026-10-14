package com.google.ar.core.examples.java.helloar;

import android.app.Activity;
import android.app.Instrumentation;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import java.io.File;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/** Explicit accidental-start guard, not a security authentication mechanism. */
public final class MeasurementInstrumentation extends Instrumentation {
  static volatile Bundle pending;
  static volatile CountDownLatch completed;
  private Bundle arguments;
  @Override public void onCreate(Bundle args) { arguments = args; start(); }
  private static String required(Bundle args, String key) {
    String value = args.getString(key);
    if (value == null || value.isEmpty()) throw new IllegalArgumentException("missing " + key);
    return value;
  }
  private static boolean flag(Bundle args, String key) {
    String value = required(args, key);
    if (!value.equals("true") && !value.equals("false")) throw new IllegalArgumentException("invalid " + key);
    return value.equals("true");
  }
  private static String safeId(Bundle args, String key) {
    String value = required(args, key);
    if (!value.matches("[A-Za-z0-9][A-Za-z0-9_-]{0,63}")) throw new IllegalArgumentException("invalid " + key);
    return value;
  }
  static synchronized Bundle consume() {
    Bundle value = pending;
    pending = null;
    return value;
  }
  @Override public void onStart() {
    Bundle result = new Bundle();
    File ownedRun = null;
    try {
      if (arguments == null || !BuildGuard.TOKEN.equals(required(arguments, "token"))) throw new IllegalArgumentException("invalid token");
      String mode = required(arguments, "mode");
      if (!mode.equals("live") && !mode.equals("playback")) throw new IllegalArgumentException("invalid mode");
      int duration = Integer.parseInt(required(arguments, "durationSeconds"));
      if (duration < 1 || duration > 30) throw new IllegalArgumentException("duration must be 1..30 seconds");
      String runId = safeId(arguments, "runId");
      boolean images = flag(arguments, "saveImages");
      boolean imu = flag(arguments, "saveImu");
      boolean recording = flag(arguments, "saveArcoreRecording");
      File root = new File(getTargetContext().getFilesDir(), "reference-runs");
      File directory = new File(root, runId);
      if (directory.exists()) throw new IllegalArgumentException("runId already exists");
      Bundle config = new Bundle();
      if (mode.equals("playback")) {
        if (imu || recording) throw new IllegalArgumentException("playback cannot save live IMU or a new MP4");
        String sourceRun = safeId(arguments, "sourceRunId");
        File mp4 = new File(new File(root, sourceRun), "recording.mp4");
        if (!mp4.isFile() || mp4.length() == 0) throw new IllegalArgumentException("owned playback MP4 absent");
        config.putString("sourceRunId", sourceRun);
      }
      if (getTargetContext().checkSelfPermission(android.Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) throw new IllegalArgumentException("CAMERA permission must already be granted after measurement cue");
      if (!root.exists() && !root.mkdirs()) throw new IllegalStateException("cannot create runs directory");
      if (!directory.mkdir()) throw new IllegalStateException("cannot exclusively create run directory");
      ownedRun = directory;
      config.putString("runId", runId);
      config.putString("mode", mode);
      config.putInt("durationSeconds", duration);
      config.putBoolean("saveImages", images);
      config.putBoolean("saveImu", imu);
      config.putBoolean("saveArcoreRecording", recording);
      pending = config;
      completed = new CountDownLatch(1);
      startActivitySync(new Intent(getTargetContext(), HelloArActivity.class).addFlags(Intent.FLAG_ACTIVITY_NEW_TASK));
      boolean done = completed.await(duration + 15L, TimeUnit.SECONDS);
      result.putString("status", done ? "finished" : "lifecycle_timeout");
      result.putString("runId", runId);
      result.putString("summary", "files/reference-runs/" + runId + "/summary.json");
      finish(done ? Activity.RESULT_OK : Activity.RESULT_CANCELED, result);
    } catch (Exception error) {
      pending = null;
      if (ownedRun != null && !new File(ownedRun, "summary.json").exists()) {
        try (java.io.PrintWriter out = new java.io.PrintWriter(new File(ownedRun, "summary.json"))) {
          org.json.JSONObject summary = new org.json.JSONObject();
          summary.put("event", "summary");
          summary.put("reason", "instrumentation_setup_error");
          summary.put("error", error.toString());
          summary.put("frames", 0);
          out.println(summary.toString());
        } catch (Exception writeError) { result.putString("summaryWriteError", writeError.toString()); }
      }
      result.putString("status", "rejected");
      result.putString("reason", error.toString());
      finish(Activity.RESULT_CANCELED, result);
    }
  }
}
