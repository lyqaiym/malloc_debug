package com.malloc.test;

import java.io.File;

public class MallocInit {

    static {
        System.loadLibrary("malloc_debug");
    }

    public static void javaInit(File file) {
        init(file.getPath());
    }

    private static native void init(String file);

    public static native void testMalloc(int type);

    public static native void printf(String file);
}
