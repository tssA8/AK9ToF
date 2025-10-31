package dev.app.tof

import android.content.Context
import android.content.res.Resources

class RawToFSource(
    private val ctx: Context,
    private val width: Int = 120,
    private val height: Int = 90
) : ToFSource {

    private var used = false

    override fun nextFrame(): ToFFrame? {
        if (used) return null

        val res = ctx.resources
        val depth = readRawToIntArray(res, R.raw.depth1, width, height)
        val amp   = readRawToIntArray(res, R.raw.amp_1, width, height)

        used = true
        return ToFFrame(
            depth = depth,
            amp = amp,
            width = width,
            height = height,
            timestampNanos = System.nanoTime()   // ← 補這個
        )
    }


    private fun readRawToIntArray(
        res: Resources,
        rawId: Int,
        w: Int,
        h: Int
    ): IntArray {
        val input = res.openRawResource(rawId)
        val text = input.bufferedReader().use { it.readText() }
        val arr = IntArray(w * h)
        var p = 0
        text.lineSequence().forEach { line ->
            val parts = line.trim()
                .split(" ", "\t", ",")
                .filter { it.isNotEmpty() }
            for (v in parts) {
                if (p >= arr.size) break
                // 這裡用 toInt() 就不會爆了
                val value = v.toInt()
                arr[p++] = value
            }
        }
        return arr
    }
}
