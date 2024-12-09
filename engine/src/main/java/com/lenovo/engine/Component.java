package com.lenovo.engine;

/**
 * Create by xuezhiyuan on 2024/12/3
 */

public abstract class Component {
    private static boolean libraryFound = false;

    static {
        try {
            System.loadLibrary("engine");
            libraryFound = true;
        } catch (UnsatisfiedLinkError e) {
            e.printStackTrace();
        }
    }

    public abstract long createInstance();

    public abstract void destroy();

    public static class Library {
        public static boolean isLibraryFound() {
            return libraryFound;
        }

        public static String getTag() {
            return "Component";
        }
    }
}
