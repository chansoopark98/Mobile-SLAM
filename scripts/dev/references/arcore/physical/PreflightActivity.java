package com.google.ar.core.examples.java.helloar;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

/** Launching this screen does not start a camera, sensor, session, or recording. */
public final class PreflightActivity extends Activity {
  @Override public void onCreate(Bundle state) {
    super.onCreate(state);
    TextView text = new TextView(this);
    text.setPadding(32, 64, 32, 32);
    text.setText("Mobile-SLAM ARCore reference\n\nReady. Measurement has not started.\nCamera permission and an explicit measurement cue are required.\n\nThis screen does not collect camera images or sensor data.");
    setContentView(text);
  }
}
