package dev.app.tof.io

import android.content.Context
import android.content.res.Resources
import dev.app.tof.core.ToFFrame

class RawToFSource(
    private val ctx: Context,
    private val depthRawId: Int,
    private val ampRawId: Int,
    private val width: Int = 120,
    private val height: Int = 90
) : ToFSource {

    private var used = false

    override fun nextFrame(): ToFFrame? {
        if (used) return null

        val res = ctx.resources
        val depth = readRawToIntArray(res, depthRawId, width, height)
        val amp   = readRawToIntArray(res, ampRawId,   width, height)

        used = true
        return ToFFrame(
            width = width,
            height = height,
            depth = depth,
            amp = amp,
            timestampNanos = System.nanoTime()
        )
    }

    private fun readRawToIntArray(
        res: Resources,
        rawId: Int,
        w: Int,
        h: Int
    ): IntArray {
        val need = w * h
        val out = IntArray(need)
        var p = 0
        val sep = Regex("[,\\s]+")

        res.openRawResource(rawId).bufferedReader().useLines { lines ->
            lines.forEach { line ->
                if (p >= need) return@forEach
                val s = line.trim().removePrefix("\uFEFF") // 去 BOM
                if (s.isEmpty()) return@forEach
                s.split(sep).forEach { tok ->
                    if (tok.isEmpty() || p >= need) return@forEach
                    // 若遇到非數字，直接略過或拋錯都可；這裡選擇拋錯讓資料更嚴謹
                    val v = tok.toIntOrNull()
                        ?: throw IllegalArgumentException("Non-integer token '$tok' in rawId=$rawId")
                    out[p++] = v
                }
            }
        }

        require(p == need) {
            "Raw data size mismatch for rawId=$rawId: read=$p, expected=$need (width=$w, height=$h)"
        }
        return out
    }
}
