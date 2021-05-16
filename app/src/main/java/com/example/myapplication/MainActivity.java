package com.example.myapplication;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;

import com.bytedance.raphael.Raphael;
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

        String space = new File(getExternalCacheDir(), "raphael").getAbsolutePath();
        Raphael.start(Raphael.MAP64_MODE|Raphael.ALLOC_MODE|0x0F0000|1024, space, null);

        findViewById(R.id.bt).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                new Thread() {
                    @Override
                    public void run() {
                        int size = 10000;
                        long b = System.currentTimeMillis();
                        for (int i = 0; i < size; i++) {
                            MallocInit.testMalloc(2);
                        }
                        Log.d(TAG, "test_time3 Raphael_2 " + (System.currentTimeMillis() - b));
                        b = System.currentTimeMillis();
                        for (int i = 0; i < size; i++) {
                            MallocInit.testMalloc(1);
                        }
                        Log.d(TAG, "test_time4 Raphael_1 " + (System.currentTimeMillis() - b));
                        b = System.currentTimeMillis();
                        for (int i = 0; i < size; i++) {
                            MallocInit.testMalloc(0);
                        }
                        Log.d(TAG, "test_time5 Raphael_0 " + (System.currentTimeMillis() - b));
                        File file = new File(dir, "malloc_printf.txt");
                        MallocInit.printf(file.getPath());
                    }
                }.start();
            }
        });
        findViewById(R.id.bt2).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Raphael.print();
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        isStart = false;
    }
}