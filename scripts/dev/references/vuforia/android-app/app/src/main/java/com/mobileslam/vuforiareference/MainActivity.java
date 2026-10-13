package com.mobileslam.vuforiareference;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

public final class MainActivity extends Activity {
    static { System.loadLibrary("vuforia_reference"); }
    private native String sdkVersion();

    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        TextView text = new TextView(this);
        text.setPadding(24, 24, 24, 24);
        text.setText(sdkVersion());
        setContentView(text);
    }
}
