package dev.app.tof.io

import dev.app.tof.core.ToFFrame
import java.io.File

class FileToFSource(
    private val folder: File,
    private val width: Int = 120,
    private val height: Int = 90
) : ToFSource {

    private var index = 1

    override fun nextFrame(): ToFFrame? {
        val depthFile = File(folder, "Depth(mm)_$index.txt")
        val ampFile   = File(folder, "Amp_$index.txt")
        if (!depthFile.exists() || !ampFile.exists()) return null

        // 這裡改成 Int
        val depth = readTxtToIntArray(depthFile, width, height)
        val amp   = readTxtToIntArray(ampFile, width, height)

        index++

        // 這裡就能塞進去，因為 ToFFrame 也是 IntArray
        return ToFFrame(
            depth = depth,
            amp = amp,
            width = width,
            height = height
        )
    }

    private fun readTxtToIntArray(f: File, w: Int, h: Int): IntArray {
        val lines = f.readLines()
        val arr = IntArray(w * h)
        var p = 0
        for (line in lines) {
            val parts = line.trim()
                .split(" ", "\t", ",")  // 多給幾個分隔
                .filter { it.isNotEmpty() }
            for (v in parts) {
                if (p >= arr.size) break
                arr[p++] = v.toInt()    // ← 不要 toShort()，65500 也吃得下
            }
        }
        return arr
    }
}
