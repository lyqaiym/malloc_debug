package com.example.myapplication;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.util.Log;
import android.view.View;

import com.malloc.test.MallocInit;

import java.io.File;

public class MainActivity extends AppCompatActivity {
    String TAG = "MainActivityLog";
    boolean isStart = true;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        File dir = new File(getExternalCacheDir(), "aaa");
        if (!dir.exists()) {
            dir.mkdirs();
        }
        File file = new File(dir, "malloc_init.txt");
        MallocInit.javaInit(file);
        findViewById(R.id.bt).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                new Thread() {
                    @Override
                    public void run() {
                        long b = System.currentTimeMillis();
                        for (int i = 0; i < 10000; i++) {
                            MallocInit.test();
                        }
                        Log.d(TAG, "test_time=" + (System.currentTimeMillis() - b));
                        File file = new File(dir, "malloc_printf.txt");
                        MallocInit.printf(file.getPath());
                    }
                }.start();
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        isStart = false;
    }
}