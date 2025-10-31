package dev.app.tof

import java.io.File

class FileToFSource(
    private val folder: File,        // 你 dump txt 的資料夾
    private val width: Int = 120,
    private val height: Int = 90
) : ToFSource {

    private var index = 0

    override fun nextFrame(): ToFFrame? {
        val depthFile = File(folder, "Depth(mm)_$index.txt")
        val ampFile   = File(folder, "Amp_$index.txt")
        if (!depthFile.exists() || !ampFile.exists()) return null

        val depth = readTxtToShortArray(depthFile, width, height)
        val amp   = readTxtToShortArray(ampFile, width, height)

        index++
        return ToFFrame(depth, amp, width, height)
    }

    private fun readTxtToShortArray(f: File, w: Int, h: Int): ShortArray {
        val lines = f.readLines()
        val arr = ShortArray(w * h)
        var p = 0
        for (line in lines) {
            val parts = line.trim().split(" ", "\t").filter { it.isNotEmpty() }
            for (v in parts) {
                arr[p++] = v.toShort()
            }
        }
        return arr
    }
}
