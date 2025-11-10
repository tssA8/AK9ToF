package dev.app.tof

import java.io.*
import kotlin.math.*

/** 依內參建 Ray LUT（dirX=x/z, dirY=y/z），可快取到檔案以加速下次啟動。 */
object LutBuilder {

    /** 建立 LUT（會把畸變校正算進去；若 K.rectified=true 則視為無畸變） */
    fun build(K: Intrinsics): RaysLUT = RaysLUT(K)

    /** 以內參建立對應檔名，避免內參/尺寸變動時誤用 */
    fun cacheFile(baseDir: File, K: Intrinsics): File {
        val key = "${K.width}x${K.height}_" +
                "${K.fx}_${K.fy}_${K.cx}_${K.cy}_" +
                "${K.k1}_${K.k2}_${K.k3}_${K.p1}_${K.p2}_" +
                "rect${K.rectified}"
        val hash = key.hashCode()
        return File(baseDir, "raylut_$hash.bin")
    }

    /** 嘗試從快取載入；失敗則回傳 null */
    fun load(file: File, K: Intrinsics): RaysLUT? = try {
        DataInputStream(BufferedInputStream(FileInputStream(file))).use { inp ->
            val w = inp.readInt(); val h = inp.readInt()
            if (w != K.width || h != K.height) return null
            val lut = RaysLUT(K)
            inp.readFully(FloatArray(w*h).also { arr ->
                for (i in arr.indices) arr[i] = inp.readFloat()
                System.arraycopy(arr, 0, lut.dirX, 0, arr.size)
            }.let { ByteArray(0) }) // 讓 readFully 完整讀取（無用返回）
            // 再讀 dirY
            for (i in 0 until w*h) lut.dirY[i] = inp.readFloat()
            lut
        }
    } catch (_: Throwable) { null }

    /** 存成快取檔（小檔案，120x90 約 86KB×2） */
    fun save(file: File, lut: RaysLUT) {
        val w = lut.K.width; val h = lut.K.height
        DataOutputStream(BufferedOutputStream(FileOutputStream(file))).use { out ->
            out.writeInt(w); out.writeInt(h)
            for (i in 0 until w*h) out.writeFloat(lut.dirX[i])
            for (i in 0 until w*h) out.writeFloat(lut.dirY[i])
        }
    }

}
