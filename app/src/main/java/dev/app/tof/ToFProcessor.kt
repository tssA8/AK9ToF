package dev.app.tof

import android.util.Log
import java.util.concurrent.ArrayBlockingQueue
import kotlin.math.acos
import kotlin.math.sqrt

data class Debug3DPoint(
    val u: Int,
    val v: Int,
    val depthMm: Int,
    val amp: Int,
    val x: Float,
    val y: Float,
    val z: Float
)

data class CalibrationResult(
    val valid: Boolean,
    val pointsCount: Int,
    val samplePoints: List<Debug3DPoint> = emptyList()
)

class ToFProcessor(
    private val listener: (CalibrationResult) -> Unit
) {
    private val queue = ArrayBlockingQueue<ToFFrame>(3)
    @Volatile private var running = false
    private var worker: Thread? = null

    fun start() {
        if (running) return
        running = true
        worker = Thread {
            while (running && !Thread.currentThread().isInterrupted) {
                val frame = try {
                    queue.take()
                } catch (_: InterruptedException) {
                    break
                }
                val result = processFrame(frame)
                listener(result)
            }
        }.apply { start() }
    }

    fun stop() {
        running = false
        worker?.interrupt()
        worker = null
    }

    fun submit(frame: ToFFrame) {
        if (!queue.offer(frame)) {
            // 滿了就丟最舊的
            queue.poll()
            queue.offer(frame)
        }
    }

    // ----------------------------------------------------
    // 1. 做前處理 / ROI / depth 範圍
    // ----------------------------------------------------
    private fun buildValidMask(frame: ToFFrame): BooleanArray {
        val w = frame.width
        val h = frame.height
        val mask = BooleanArray(w * h)

        val zMinMm = 450   // 0.45m 以下當雜訊
        val zMaxMm = 4000  // 4m 以上不要

        for (v in 0 until h) {
            for (u in 0 until w) {
                val i = v * w + u
                val d = frame.depth[i]
                val a = frame.amp[i]

                val goodDepth = d in zMinMm..zMaxMm
                val goodAmp = (a != 0 && a < 65000)

                // 這裡如果你要再加畫面 ROI，可以打開這個：
                // val inRect = u in 5..114 && v in 3..86
                // mask[i] = goodDepth && goodAmp && inRect

                mask[i] = goodDepth && goodAmp
            }
        }
        return mask
    }

    // ----------------------------------------------------
    // 2. 一幀的主流程
    // ----------------------------------------------------
    private fun processFrame(frame: ToFFrame): CalibrationResult {
        val mask = buildValidMask(frame)
        val points = depthAmpTo3D(frame, mask)

        // 👉 新增：用這些 3D 點估平面
        val plane = estimatePlane(points)

        if (plane != null) {
            var n = plane.normal  // (nx, ny, nz)

            // 👉 這一行是重點：如果平面是背對相機，就把它轉到面向相機
            if (n[2] < 0f) {
                n = floatArrayOf(-n[0], -n[1], -n[2])
            }

            val camForward = floatArrayOf(0f, 0f, 1f)
            val dot = (n[0] * camForward[0] +
                    n[1] * camForward[1] +
                    n[2] * camForward[2]).coerceIn(-1f, 1f)

            val angleRad = kotlin.math.acos(dot)
            val angleDeg = Math.toDegrees(angleRad.toDouble())

            Log.d("ToF", "plane normal = (${n[0]}, ${n[1]}, ${n[2]}), tilt = $angleDeg deg, d=${plane.d}")
        }

        val sample = if (points.size > 5) points.subList(0, 5) else points
        return CalibrationResult(
            valid = points.isNotEmpty(),
            pointsCount = points.size,
            samplePoints = sample
        )
    }

    // ----------------------------------------------------
    // 3. 平面資料
    // ----------------------------------------------------
    data class Plane(
        val normal: FloatArray,  // 長度 3，已經 normalize
        val d: Float             // ax + by + cz + d = 0 的 d
    )


// ----------------------------------------------------
// 4. 用最小平方做 z = ax + by + c 的平面
// ----------------------------------------------------
    private fun estimatePlane(points: List<Debug3DPoint>): Plane? {
        if (points.size < 3) return null

        // 我們要解的式子是：
        // z = a*x + b*y + c
        //
        // 正規方程會變成：
        // [ Σx²  Σxy  Σx ] [a]   [ Σxz ]
        // [ Σxy  Σy²  Σy ] [b] = [ Σyz ]
        // [ Σx   Σy   N  ] [c]   [ Σz  ]

        var sumX = 0.0
        var sumY = 0.0
        var sumZ = 0.0
        var sumXX = 0.0
        var sumYY = 0.0
        var sumXY = 0.0
        var sumXZ = 0.0
        var sumYZ = 0.0

        for (p in points) {
            val x = p.x.toDouble()
            val y = p.y.toDouble()
            val z = p.z.toDouble()

            sumX += x
            sumY += y
            sumZ += z
            sumXX += x * x
            sumYY += y * y
            sumXY += x * y
            sumXZ += x * z
            sumYZ += y * z
        }

        val n = points.size.toDouble()

        // 先組 2x2，先解 a, b，再解 c
        // [Sxx Sxy][a] = [Sxz]
        // [Sxy Syy][b]   [Syz]
        val Sxx = sumXX
        val Syy = sumYY
        val Sxy = sumXY
        val Sxz = sumXZ
        val Syz = sumYZ

        val det = Sxx * Syy - Sxy * Sxy
        if (kotlin.math.abs(det) < 1e-8) {
            // 幾何上退化了，直接回傳正面
            return Plane(floatArrayOf(0f, 0f, 1f), (-sumZ / n).toFloat())
        }

        val a = ( Syy * Sxz - Sxy * Syz ) / det
        val b = ( Sxx * Syz - Sxy * Sxz ) / det
        // c 用平均值算：z̄ = a x̄ + b ȳ + c
        val xBar = sumX / n
        val yBar = sumY / n
        val zBar = sumZ / n
        val c = zBar - a * xBar - b * yBar

        // 平面：z = a x + b y + c
        // 轉成 ax + by - z + c = 0
        // 法向量 = (a, b, -1)
        val nx = a.toFloat()
        val ny = b.toFloat()
        val nz = -1f

        val len = kotlin.math.sqrt(nx*nx + ny*ny + nz*nz)
        val nxf = nx / len
        val nyf = ny / len
        val nzf = nz / len

        // ax + by + cz + d = 0 → d = c (因為我們是 ax + by - z + c = 0)
        val d = c.toFloat()

        return Plane(floatArrayOf(nxf, nyf, nzf), d)
    }


    // ----------------------------------------------------
    // 5. 這個是你剛剛少的：找「最小特徵向量」的 helper
    // 這裡我給你一個超簡版：直接比三個方向的變異量，挑最小的當法向量
    // 因為你現在的資料就是「一張牆」，這樣用是 OK 的
    // ----------------------------------------------------
    private fun smallestEigenVector3(m: Array<DoubleArray>): FloatArray? {
        // m 是對稱的 3x3 協方差矩陣
        val varX = m[0][0]
        val varY = m[1][1]
        val varZ = m[2][2]

        // 找變異量最小的軸，當作法向量的「主要方向」
        return when {
            varX <= varY && varX <= varZ -> floatArrayOf(1f, 0f, 0f)
            varY <= varX && varY <= varZ -> floatArrayOf(0f, 1f, 0f)
            else -> floatArrayOf(0f, 0f, 1f)
        }
    }

    // ----------------------------------------------------
    // 6. 2D → 3D
    // ----------------------------------------------------
    private fun depthAmpTo3D(frame: ToFFrame, mask: BooleanArray): List<Debug3DPoint> {
        val w = frame.width
        val h = frame.height
        val fx = ToFIntrinsics.FX
        val fy = ToFIntrinsics.FY
        val cx = ToFIntrinsics.CX
        val cy = ToFIntrinsics.CY

        val out = ArrayList<Debug3DPoint>()
        for (v in 0 until h) {
            for (u in 0 until w) {
                val idx = v * w + u
                if (!mask[idx]) continue

                val dmm = frame.depth[idx]
                val amp = frame.amp[idx]
                val z = dmm / 1000f
                val X = (u - cx) * z / fx
                val Y = (v - cy) * z / fy

                out.add(
                    Debug3DPoint(
                        u = u,
                        v = v,
                        depthMm = dmm,
                        amp = amp,
                        x = X,
                        y = Y,
                        z = z
                    )
                )
            }
        }
        return out
    }
}
