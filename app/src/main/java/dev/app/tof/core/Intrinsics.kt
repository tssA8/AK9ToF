package dev.app.tof.core

// ==============================
//  內參資料結構與工具
// ==============================
data class Intrinsics(
    val fx: Float,
    val fy: Float,
    val cx: Float,
    val cy: Float,
    val k1: Float = 0f,
    val k2: Float = 0f,
    val p1: Float = 0f,
    val p2: Float = 0f,
    val k3: Float = 0f,
    val width: Int,
    val height: Int,
    val rectified: Boolean = false  // 若影像已做 undistort/rectify，畸變視為 0
)

/**
 * 你的「母參數」來自同事給的 640x480 JSON。
 * 若確定 ToF SDK 已做過整流，可在 deriveForSize() 以 forceRectified=true 把畸變係數歸零。
 */
object CalibRepo {
    // 你的同事給的 JSON（640x480）
    val base640 = Intrinsics(
        fx = 276.1183117f, fy = 275.8061159f,
        cx = 268.1144804f, cy = 234.7569998f,
        k1 = 0.06991161f, k2 = -0.11384410f, p1 = -0.0002503818f, p2 = 0.0015746408f, k3 = 0.026850856f,
        width = 640, height = 480,
        rectified = false
    )

    /**
     * 將 base 內參推導到目標影像大小。
     * 若先裁切(以 base 解析度為座標)，再縮放到 newW/newH：
     *   1) 先對 cx,cy 扣掉裁切偏移
     *   2) 再乘縮放比例
     */
    fun deriveForSize(
        base: Intrinsics = base640,
        newW: Int, newH: Int,
        cropLeft: Int = 0, cropTop: Int = 0, cropW: Int? = null, cropH: Int? = null,
        forceRectified: Boolean? = null
    ): Intrinsics {
        val srcW = cropW ?: base.width
        val srcH = cropH ?: base.height
        val sx = newW.toFloat() / srcW
        val sy = newH.toFloat() / srcH

        val cxCropped = base.cx - cropLeft
        val cyCropped = base.cy - cropTop

        val fx = base.fx * sx
        val fy = base.fy * sy
        val cx = cxCropped * sx
        val cy = cyCropped * sy

        val rectified = forceRectified ?: base.rectified
        return Intrinsics(
            fx, fy, cx, cy,
            k1 = if (rectified) 0f else base.k1,
            k2 = if (rectified) 0f else base.k2,
            p1 = if (rectified) 0f else base.p1,
            p2 = if (rectified) 0f else base.p2,
            k3 = if (rectified) 0f else base.k3,
            width = newW, height = newH,
            rectified = rectified
        )
    }
}

/** 像素(u,v) → 去畸變後的歸一座標 (x/z, y/z) */
fun undistortToNormalized(u: Float, v: Float, K: Intrinsics): Pair<Float, Float> {
    var x = (u - K.cx) / K.fx
    var y = (v - K.cy) / K.fy
    if (!K.rectified) {
        val r2 = x * x + y * y
        val radial = 1f + K.k1 * r2 + K.k2 * r2 * r2 + K.k3 * r2 * r2 * r2
        val x_tan = 2f * K.p1 * x * y + K.p2 * (r2 + 2f * x * x)
        val y_tan = K.p1 * (r2 + 2f * y * y) + 2f * K.p2 * x * y
        x = x * radial + x_tan
        y = y * radial + y_tan
    }
    return x to y
}

/** 方向 LUT：對每個像素存 X/Z、Y/Z，2D→3D 時只需乘上 z */
class RaysLUT(val K: Intrinsics) {
    val dirX = FloatArray(K.width * K.height)
    val dirY = FloatArray(K.width * K.height)

    init {
        for (v in 0 until K.height) {
            for (u in 0 until K.width) {
                val (nx, ny) = undistortToNormalized(u.toFloat(), v.toFloat(), K)
                val i = v * K.width + u
                dirX[i] = nx
                dirY[i] = ny
            }
        }
    }
}