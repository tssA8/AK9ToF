package dev.app.tof

data class ToFFrame(
    val depth: ShortArray,   // 長度 = width*height
    val amp: ShortArray,     // 同長度
    val width: Int,
    val height: Int,
    val timestampNanos: Long = System.nanoTime()
)