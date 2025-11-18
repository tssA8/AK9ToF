// PcdReader.kt
package dev.app.tof.io

import android.content.Context
import java.io.BufferedReader
import java.io.InputStreamReader
import kotlin.math.min

data class PcdCloud(val x: FloatArray, val y: FloatArray, val z: FloatArray)

object PcdReader {
    /** 只支援 ASCII PCD，欄位包含 x y z */
    fun readAsciiFromRaw(ctx: Context, resId: Int, maxPoints: Int = Int.MAX_VALUE): PcdCloud {
        ctx.resources.openRawResource(resId).use { ins ->
            BufferedReader(InputStreamReader(ins)).use { br ->
                // --- header ---
                var line: String
                var width = -1
                var height = 1
                var dataAscii = false
                val header = mutableListOf<String>()
                while (true) {
                    line = br.readLine() ?: error("Unexpected EOF in PCD header")
                    header.add(line)
                    if (line.startsWith("WIDTH"))   width = line.split("\\s+".toRegex())[1].toInt()
                    if (line.startsWith("HEIGHT"))  height = line.split("\\s+".toRegex())[1].toInt()
                    if (line.startsWith("DATA ascii")) { dataAscii = true; break }
                }
                require(dataAscii) { "Only DATA ascii supported" }

                val n = min((if (width>0) width else Int.MAX_VALUE) * height, maxPoints)
                val xs = ArrayList<Float>(n)
                val ys = ArrayList<Float>(n)
                val zs = ArrayList<Float>(n)

                // --- data ---
                var count = 0
                while (true) {
                    val dl = br.readLine() ?: break
                    val tok = dl.trim().split(Regex("\\s+"))
                    if (tok.size < 3) continue
                    xs.add(tok[0].toFloat())
                    ys.add(tok[1].toFloat())
                    zs.add(tok[2].toFloat())
                    count++
                    if (count >= n) break
                }
                return PcdCloud(
                    x = FloatArray(xs.size) { xs[it] },
                    y = FloatArray(ys.size) { ys[it] },
                    z = FloatArray(zs.size) { zs[it] }
                )
            }
        }
    }
}
