package dev.app.tof

import android.content.Context
import android.content.res.Resources

// 你現在只有一幀，所以先做一幀版
class RawToFSource(
    private val ctx: Context,
    private val width: Int = 120,
    private val height: Int = 90
) : ToFSource {

    private var used = false   // 先讀一次就好

    override fun nextFrame(): ToFFrame? {
        if (used) return null

        val res = ctx.resources

        // 對應你現在放的檔名：
        // res/raw/depth1.txt  -> R.raw.depth1
        // res/raw/amp_1.txt   -> R.raw.amp_1
        val depth = readRawToShortArray(res, R.raw.depth1, width, height)
        val amp   = readRawToShortArray(res, R.raw.amp_1, width, height)

        used = true
        return ToFFrame(depth, amp, width, height)
    }

    private fun readRawToShortArray(
        res: Resources,
        rawId: Int,
        w: Int,
        h: Int
    ): ShortArray {
        val input = res.openRawResource(rawId)
        val text = input.bufferedReader().use { it.readText() }
        val arr = ShortArray(w * h)
        var p = 0
        text.lineSequence().forEach { line ->
            val parts = line.trim().split(" ", "\t", ",")
                .filter { it.isNotEmpty() }
            for (v in parts) {
                if (p < arr.size) {
                    arr[p++] = v.toShort()
                } else {
                    break
                }
            }
        }
        return arr
    }
}
