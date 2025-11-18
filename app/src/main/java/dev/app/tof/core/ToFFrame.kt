package dev.app.tof.core

data class ToFFrame(
    val depth: IntArray,
    val amp: IntArray,
    val width: Int,
    val height: Int,
    val timestampNanos: Long = System.nanoTime()  // ← 給預設
)
