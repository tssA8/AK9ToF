package dev.app.tof

import android.util.Log
import java.io.File
import java.util.concurrent.ArrayBlockingQueue
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.sqrt

// ----------------------------------------------------
// 資料結構（保持你原本結構 + 新增 PlaneFitInfo）
// ----------------------------------------------------
data class Debug3DPoint(
    val u: Int,
    val v: Int,
    val depthMm: Int,
    val amp: Int,
    val x: Float,
    val y: Float,
    val z: Float
)

/** 對外提供完整的平面資訊 */
data class PlaneFitInfo(
    val nx: Float, val ny: Float, val nz: Float,  // 單位：無量綱
    val dMeters: Float,                           // 平面 ax+by+cz+d=0 的 d（單位 m）
    val dMm: Float,                               // d 的 mm 版本（方便跟 PCD/同事對齊）
    val tiltDeg: Double,
    val pitchDeg: Double,
    val yawDeg: Double,
    val rmseMm: Double,
    val inliers: Int,
    val total: Int
) {
    val inlierRatio: Double get() = if (total > 0) inliers.toDouble() / total else 0.0
}

data class CalibrationResult(
    val valid: Boolean,
    val pointsCount: Int,
    val samplePoints: List<Debug3DPoint> = emptyList(),

    // === 舊欄位（相容）===
    val pitchDeg: Double? = null,
    val yawDeg: Double? = null,
    val tiltDeg: Double? = null,
    val rmseMm: Double? = null,
    val inlierRatio: Double? = null,

    // === 新增：完整平面資訊 ===
    val plane: PlaneFitInfo? = null
)


// ==============================
//  ToFProcessor（改：可選 cacheDir + 對外平面資訊）
// ==============================
class ToFProcessor(
    private val listener: (CalibrationResult) -> Unit,
    private val cacheDir: File? = null   // ← 給就做 LUT 快取；不給就只在記憶體建
) {
    private val queue = ArrayBlockingQueue<ToFFrame>(3)
    @Volatile private var running = false
    private var worker: Thread? = null

    // 動態推導後的內參與 LUT（依第一幀/尺寸變化建立）
    private var activeK: Intrinsics? = null
    private var rays: RaysLUT? = null
    private var printedIntrinsics = false

    fun start() {
        if (running) return
        running = true
        worker = Thread {
            while (running && !Thread.currentThread().isInterrupted) {
                val frame = try { queue.take() } catch (_: InterruptedException) { break }
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
            queue.poll()
            queue.offer(frame)
        }
    }

    // ----------------------------------------------------
    // 主流程
    // ----------------------------------------------------
    private fun processFrame(frame: ToFFrame): CalibrationResult {
        val w = frame.width
        val h = frame.height
        if (w <= 0 || h <= 0 || frame.depth.size < w * h || frame.amp.size < w * h) {
            Log.w(TAG, "invalid frame")
            return CalibrationResult(valid = false, pointsCount = 0)
        }

        // 0) 建立/更新當前解析度的內參與 LUT（以 640x480 為母參數換算）
        ensureIntrinsicsAndRays(w, h)

        // 0.1) 內參健檢（只印一次）
        sanityPrintIntrinsicsOnce()

        // 1) 參數
        val zMinMm = 450
        val zMaxMm = 4000
        val minAmp = 100
        val binSizeMm = 10
        val centerRectRatio = 0.6f

        // 2) 3×3 中位數濾波
        val depthMedian = median3(frame.depth, w, h)

        // 3) 直方圖抓主峰（先做基本過濾）
        val binsCount = (zMaxMm - zMinMm) / binSizeMm + 1
        val bins = IntArray(max(1, binsCount))
        val total = w * h
        for (i in 0 until total) {
            val d = depthMedian[i]
            val a = frame.amp[i]
            if (d in zMinMm..zMaxMm && a >= minAmp && a < 65000) {
                val idx = (d - zMinMm) / binSizeMm
                if (idx in bins.indices) bins[idx]++
            }
        }
        val peakIdx = bins.indices.maxBy { bins[it] }
        val peakMm = zMinMm + peakIdx * binSizeMm
        val deltaMm = maxOf(30, (peakMm * 0.015f).toInt())

        // 4) 主峰 ROI + 幾何 ROI + 門檻
        val uMargin = ((1 - centerRectRatio) / 2f)
        val vMargin = ((1 - centerRectRatio) / 2f)
        val uMin = (w * uMargin).toInt()
        val uMax = (w * (1f - uMargin)).toInt() - 1
        val vMin = (h * vMargin).toInt()
        val vMax = (h * (1f - vMargin)).toInt() - 1

        val mask = BooleanArray(total) { idx ->
            val d = depthMedian[idx]
            val a = frame.amp[idx]
            if (!(d in zMinMm..zMaxMm)) return@BooleanArray false
            if (!(a >= minAmp && a < 65000)) return@BooleanArray false
            if (abs(d - peakMm) > deltaMm) return@BooleanArray false
            val u = idx % w
            val v = idx / w
            if (u !in uMin..uMax || v !in vMin..vMax) return@BooleanArray false
            true
        }

        // 5) 2D → 3D（用 LUT）
        val raysLocal = rays ?: return CalibrationResult(valid = false, pointsCount = 0)
        val points = ArrayList<Debug3DPoint>()
        points.ensureCapacity(total / 2)
        for (vv in 0 until h) {
            val rowOffset = vv * w
            for (uu in 0 until w) {
                val idx = rowOffset + uu
                if (!mask[idx]) continue
                val dmm = depthMedian[idx]
                val amp = frame.amp[idx]
                val z = dmm / 1000f
                val x = raysLocal.dirX[idx] * z
                val y = raysLocal.dirY[idx] * z
                points.add(Debug3DPoint(uu, vv, dmm, amp, x, y, z))
            }
        }

        if (points.isEmpty()) {
            Log.w(TAG, "no valid 3D points after ROI filter")
            return CalibrationResult(valid = false, pointsCount = 0)
        }

        latestSdkCloud?.let { (sx, sy, sz) ->
            val stats = validateAgainstSdk(rays!!, frame, sx, sy, sz)
            Log.d(TAG, "LUT-vs-SDK count=${stats.count}, rmse=${"%.1f".format(stats.rmseOverallMm)}mm, p95=${"%.1f".format(stats.p95OverallMm)}mm")
        }

        // 6) 平面擬合（RANSAC → LLS）
        val plane = estimatePlaneRansac(points) ?: run {
            Log.w(TAG, "plane fit failed")
            return CalibrationResult(valid = false, pointsCount = points.size, samplePoints = points.take(5))
        }

        // 7) 法向量翻正（+Z）
        var n = plane.normal
        var d = plane.d
        if (n[2] < 0f) {
            n = floatArrayOf(-n[0], -n[1], -n[2])
            d = -d
        }

        // 8) 角度
        val rawTilt = Math.toDegrees(acos(n[2].coerceIn(-1f, 1f).toDouble()))
        val tiltDeg = if (rawTilt > 90) 180 - rawTilt else rawTilt
        val yawDeg = Math.toDegrees(atan2(n[0].toDouble(), n[2].toDouble()))
        val pitchDeg = Math.toDegrees(atan2(-n[1].toDouble(), sqrt((n[0]*n[0] + n[2]*n[2]).toDouble())))

        // 9) 品質：RMSE / inliers（動態閾值）
        var sumSqMm = 0.0
        var inliers = 0
        fun dynamicInlierThM(zMeters: Float): Float {
            val byPercent = 0.005f * zMeters   // 0.5%
            return max(0.008f, byPercent)      // at least 8mm
        }
        for (p in points) {
            val distM = abs(n[0]*p.x + n[1]*p.y + n[2]*p.z + d)
            val distMm = distM * 1000.0
            sumSqMm += distMm * distMm
            if (distM <= dynamicInlierThM(p.z)) inliers++
        }
        val rmseMm = sqrt(sumSqMm / points.size).toFloat()
        val inlierRatio = inliers.toFloat() / points.size

        // 10) Log sample
        val sample = points.take(5)
        Log.d(
            TAG,
            "plane n=(${n[0]}, ${n[1]}, ${n[2]}), d=${"%.6f".format(d)} m (${(d*1000f).toInt()} mm), " +
                    "tilt=${"%.2f".format(tiltDeg)}°, pitch=${"%.2f".format(pitchDeg)}°, yaw=${"%.2f".format(yawDeg)}°, " +
                    "rmse=${"%.1f".format(rmseMm)}mm, inliers=${"%.1f".format(inlierRatio*100)}%, 3D points=${points.size}"
        )
        if (sample.isNotEmpty()) {
            val p0 = sample[0]
            Log.d(TAG, "pt[0] pix=(${p0.u},${p0.v}) depth=${p0.depthMm}mm amp=${p0.amp} → X=${p0.x}, Y=${p0.y}, Z=${p0.z}")
        }

        // 11) 封裝 PlaneFitInfo + 回傳
        val planeInfo = PlaneFitInfo(
            nx = n[0], ny = n[1], nz = n[2],
            dMeters = d,
            dMm = d * 1000f,
            tiltDeg = tiltDeg,
            pitchDeg = pitchDeg,
            yawDeg = yawDeg,
            rmseMm = rmseMm.toDouble(),
            inliers = inliers,
            total = points.size
        )

        return CalibrationResult(
            valid = true,
            pointsCount = points.size,
            samplePoints = sample,

            // 舊欄位（相容）
            tiltDeg = tiltDeg,
            pitchDeg = pitchDeg,
            yawDeg = yawDeg,
            rmseMm = rmseMm.toDouble(),
            inlierRatio = inlierRatio.toDouble(),

            // 新欄位
            plane = planeInfo
        )
    }

    // ==============================
    // 內參與 LUT 構建（支援可選快取）
    // ==============================
    private fun ensureIntrinsicsAndRays(w: Int, h: Int) {
        val needRebuild = (activeK == null || activeK!!.width != w || activeK!!.height != h)
        if (!needRebuild) return

        // 🔧 暫時：用 ToF 自己的 FOV 估計內參（請替換成你模組的實際 FOV）
        val fovXdeg = 60.0
        val fovYdeg = 45.0
        val fx = (w / (2.0 * kotlin.math.tan(Math.toRadians(fovXdeg/2)))).toFloat()
        val fy = (h / (2.0 * kotlin.math.tan(Math.toRadians(fovYdeg/2)))).toFloat()
        val cx = (w - 1) / 2f
        val cy = (h - 1) / 2f
        activeK = Intrinsics(
            width = w, height = h,
            fx = fx, fy = fy, cx = cx, cy = cy,
            rectified = true // ToF SDK 多半已整流；先視為 true
        )

        // 重建 LUT
        rays = RaysLUT(activeK!!)
        printedIntrinsics = false
    }



    @Volatile private var latestSdkCloud: Triple<FloatArray, FloatArray, FloatArray>? = null
    fun updateSdkCloud(x: FloatArray, y: FloatArray, z: FloatArray) {
        latestSdkCloud = Triple(x, y, z)
    }

    private fun sanityPrintIntrinsicsOnce() {
        if (printedIntrinsics) return
        val K = activeK ?: return
        val fovX = Math.toDegrees(2.0 * kotlin.math.atan(K.width / (2.0 * K.fx.toDouble())))
        val fovY = Math.toDegrees(2.0 * kotlin.math.atan(K.height / (2.0 * K.fy.toDouble())))
        Log.d(TAG, "ACTIVE INTRINSICS fx=${K.fx} fy=${K.fy} cx=${K.cx} cy=${K.cy}, " +
                "size=${K.width}x${K.height}, FOVx≈${"%.1f".format(fovX)}°, FOVy≈${"%.1f".format(fovY)}°, rectified=${K.rectified}")
        printedIntrinsics = true
    }

    // ==============================
    // 幫手：平面結構 / 擬合 / 濾波（沿用你原本）
    // ==============================
    data class Plane(val normal: FloatArray, val d: Float)

    /**
     * 使用 PCA（主成分分析）計算平面法向量。
     * 相比 LLS，更穩定、不受 z 軸比例影響。
     */
    private fun estimatePlanePca(points: List<Debug3DPoint>): Plane? {
        if (points.size < 3) return null

        // 計算中心點
        var cx = 0.0; var cy = 0.0; var cz = 0.0
        for (p in points) { cx += p.x; cy += p.y; cz += p.z }
        val n = points.size.toDouble()
        cx /= n; cy /= n; cz /= n

        // 計算共變異矩陣
        var sxx = 0.0; var sxy = 0.0; var sxz = 0.0
        var syy = 0.0; var syz = 0.0; var szz = 0.0
        for (p in points) {
            val x = p.x - cx
            val y = p.y - cy
            val z = p.z - cz
            sxx += x * x
            sxy += x * y
            sxz += x * z
            syy += y * y
            syz += y * z
            szz += z * z
        }
        sxx /= n; sxy /= n; sxz /= n
        syy /= n; syz /= n; szz /= n

        // 定義乘法：C·v
        fun mul(v: DoubleArray) = doubleArrayOf(
            sxx * v[0] + sxy * v[1] + sxz * v[2],
            sxy * v[0] + syy * v[1] + syz * v[2],
            sxz * v[0] + syz * v[1] + szz * v[2]
        )

        // 求最小特徵向量 (power iteration，25 次即可收斂)
        var v = doubleArrayOf(0.0, 0.0, 1.0)
        repeat(25) {
            val m = mul(v)
            val norm = sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]) + 1e-12
            v[0] = m[0] / norm
            v[1] = m[1] / norm
            v[2] = m[2] / norm
        }

        // 用 cross product 算出穩定法向量
        var u = doubleArrayOf(-v[1], v[0], 0.0)
        var w = mul(u)
        val dot = w[0] * u[0] + w[1] * u[1] + w[2] * u[2]
        w[0] -= dot * u[0]
        w[1] -= dot * u[1]
        w[2] -= dot * u[2]
        val wn = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) + 1e-12
        w[0] /= wn; w[1] /= wn; w[2] /= wn

        val nx = v[1] * w[2] - v[2] * w[1]
        val ny = v[2] * w[0] - v[0] * w[2]
        val nz = v[0] * w[1] - v[1] * w[0]

        var norm = sqrt(nx * nx + ny * ny + nz * nz) + 1e-12
        var n0 = floatArrayOf(
            (nx / norm).toFloat(),
            (ny / norm).toFloat(),
            (nz / norm).toFloat()
        )

        // 翻正朝向 +Z
        if (n0[2] < 0f) n0 = floatArrayOf(-n0[0], -n0[1], -n0[2])

        // 平面方程 ax + by + cz + d = 0
        val d = -(n0[0] * cx + n0[1] * cy + n0[2] * cz).toFloat()

        return Plane(n0, d)
    }


    private fun estimatePlaneRansac(points: List<Debug3DPoint>, iters: Int = 120): Plane? {
        if (points.size < 3) return null
        val rnd = java.util.Random()
        var bestPlane: Plane? = null
        var bestInliers = -1

        fun planeFrom3(p0: Debug3DPoint, p1: Debug3DPoint, p2: Debug3DPoint): Plane? {
            val ux = p1.x - p0.x; val uy = p1.y - p0.y; val uz = p1.z - p0.z
            val vx = p2.x - p0.x; val vy = p2.y - p0.y; val vz = p2.z - p0.z
            var nx = uy * vz - uz * vy
            var ny = uz * vx - ux * vz
            var nz = ux * vy - uy * vx
            val norm = sqrt(nx * nx + ny * ny + nz * nz)
            if (norm < 1e-6) return null
            nx /= norm; ny /= norm; nz /= norm
            var d = -(nx * p0.x + ny * p0.y + nz * p0.z)
            if (nz < 0f) { nx = -nx; ny = -ny; nz = -nz; d = -d }
            return Plane(floatArrayOf(nx, ny, nz), d)
        }

        // --- RANSAC 主回圈 ---
        repeat(iters) {
            val i0 = rnd.nextInt(points.size)
            var i1 = rnd.nextInt(points.size)
            var i2 = rnd.nextInt(points.size)
            while (i1 == i0) i1 = rnd.nextInt(points.size)
            while (i2 == i0 || i2 == i1) i2 = rnd.nextInt(points.size)

            val p0 = points[i0]; val p1 = points[i1]; val p2 = points[i2]
            val pl = planeFrom3(p0, p1, p2) ?: return@repeat

            var cnt = 0
            for (p in points) {
                val distM = abs(pl.normal[0] * p.x + pl.normal[1] * p.y + pl.normal[2] * p.z + pl.d)
                if (distM <= max(0.008f, 0.005f * p.z)) cnt++
            }
            if (cnt > bestInliers) {
                bestInliers = cnt
                bestPlane = pl
            }
        }

        val bp = bestPlane ?: return null
        val inlierSet = ArrayList<Debug3DPoint>(bestInliers.coerceAtLeast(0))
        for (p in points) {
            val distM = abs(bp.normal[0] * p.x + bp.normal[1] * p.y + bp.normal[2] * p.z + bp.d)
            if (distM <= max(0.008f, 0.005f * p.z)) inlierSet.add(p)
        }
        Log.d(TAG, "RANSAC inliers=${inlierSet.size}/${points.size}")

        // --- 精修 (PCA法) ---
        val refined = estimatePlanePca(inlierSet) ?: bp

        fun rmseOn(set: List<Debug3DPoint>, pl: Plane): Double {
            var s = 0.0
            for (p in set) {
                val d = abs(pl.normal[0] * p.x + pl.normal[1] * p.y + pl.normal[2] * p.z + pl.d)
                s += d * d
            }
            return if (set.isEmpty()) 1e9 else sqrt(s / set.size)
        }

        val rRansac = rmseOn(inlierSet, bp)
        val rRefine = rmseOn(inlierSet, refined)

        // --- 守門員機制 ---
        return if (rRefine <= rRansac * 1.2) refined else bp
    }


    private fun median3(a: IntArray, w: Int, h: Int): IntArray {
        val out = IntArray(a.size)
        val window = IntArray(9)
        for (y in 0 until h) {
            for (x in 0 until w) {
                var k = 0
                for (dy in -1..1) for (dx in -1..1) {
                    val xx = (x + dx).coerceIn(0, w - 1)
                    val yy = (y + dy).coerceIn(0, h - 1)
                    window[k++] = a[yy * w + xx]
                }
                window.sort()
                out[y * w + x] = window[4]
            }
        }
        return out
    }

    /** sdkX/Y/Z 長度應為 w*h；無值的點用 NaN 或 0 表示 */
    private fun validateAgainstSdk(
        rays: RaysLUT,
        frame: ToFFrame,
        sdkX: FloatArray, sdkY: FloatArray, sdkZ: FloatArray
    ): LutSdkDiffStats {
        val w = rays.K.width; val h = rays.K.height
        val n = w*h
        var cnt = 0
        var sumSq = 0.0
        var sumSqX = 0.0; var sumSqY = 0.0; var sumSqZ = 0.0
        val errs = ArrayList<Double>(n)

        for (i in 0 until n) {
            val dmm = frame.depth[i]
            val amp = frame.amp[i]
            if (dmm <= 0 || amp <= 0) continue
            val sx = sdkX[i]; val sy = sdkY[i]; val sz = sdkZ[i]
            if (!sx.isFinite() || !sy.isFinite() || !sz.isFinite()) continue

            val z = dmm / 1000f
            val x = rays.dirX[i] * z
            val y = rays.dirY[i] * z

            val dx = (x - sx).toDouble()
            val dy = (y - sy).toDouble()
            val dz = (z - sz).toDouble()
            val de = kotlin.math.sqrt(dx*dx + dy*dy + dz*dz)
            errs.add(de)
            sumSq += de*de
            sumSqX += dx*dx; sumSqY += dy*dy; sumSqZ += dz*dz
            cnt++
        }

        errs.sort()
        val p95 = if (errs.isEmpty()) 0.0 else errs[(errs.size*0.95).toInt().coerceAtMost(errs.size-1)]
        return LutSdkDiffStats(
            count = cnt,
            rmseOverallMm = if (cnt==0) 0.0 else kotlin.math.sqrt(sumSq/cnt)*1000.0,
            rmseXm = if (cnt==0) 0.0 else kotlin.math.sqrt(sumSqX/cnt),
            rmseYm = if (cnt==0) 0.0 else kotlin.math.sqrt(sumSqY/cnt),
            rmseZm = if (cnt==0) 0.0 else kotlin.math.sqrt(sumSqZ/cnt),
            p95OverallMm = p95*1000.0
        )
    }


    private fun validatePlane(points: List<Debug3DPoint>): LutPlaneMetrics? {
        if (points.size < 50) return null

        // --- 用你現有的 RANSAC + LLS ---
        val plane = estimatePlaneRansac(points) ?: return null
        var n = plane.normal; var d = plane.d
        if (n[2] < 0f) { n = floatArrayOf(-n[0], -n[1], -n[2]); d = -d }

        // 角度
        val rawTilt = Math.toDegrees(kotlin.math.acos(n[2].coerceIn(-1f, 1f).toDouble()))
        val tiltDeg = if (rawTilt > 90) 180 - rawTilt else rawTilt
        val yawDeg = Math.toDegrees(kotlin.math.atan2(n[0].toDouble(), n[2].toDouble()))
        val pitchDeg = Math.toDegrees(kotlin.math.atan2(-n[1].toDouble(), kotlin.math.sqrt((n[0]*n[0] + n[2]*n[2]).toDouble())))

        // RMSE / inliers（同你的動態閾值）
        var sumSqMm = 0.0
        var inliers = 0
        fun thM(z: Float) = maxOf(0.008f, 0.005f * z) // 8mm or 0.5%
        for (p in points) {
            val distM = kotlin.math.abs(n[0]*p.x + n[1]*p.y + n[2]*p.z + d)
            sumSqMm += (distM * 1000.0) * (distM * 1000.0)
            if (distM <= thM(p.z)) inliers++
        }
        val rmseMm = kotlin.math.sqrt(sumSqMm / points.size)
        val inlierRatio = inliers.toDouble() / points.size

        return LutPlaneMetrics(
            rmseMm = rmseMm,
            inlierRatio = inlierRatio,
            n = n,
            d = d,
            tiltDeg = tiltDeg,
            yawDeg = yawDeg,
            pitchDeg = pitchDeg,
            pointsUsed = points.size
        )
    }



    data class LutPlaneMetrics(
        val rmseMm: Double,
        val inlierRatio: Double,
        val n: FloatArray, // plane normal (nx, ny, nz) 朝 +Z
        val d: Float,
        val tiltDeg: Double,
        val yawDeg: Double,
        val pitchDeg: Double,
        val pointsUsed: Int
    )

    data class LutSdkDiffStats(
        val count: Int,
        val rmseOverallMm: Double,
        val rmseXm: Double,
        val rmseYm: Double,
        val rmseZm: Double,
        val p95OverallMm: Double
    )

    data class LutSanity(
        val w: Int, val h: Int,
        val dirXMin: Float, val dirXMax: Float, val dirXMean: Float,
        val dirYMin: Float, val dirYMax: Float, val dirYMean: Float,
        val fovXdegApprox: Double, // 2*atan(max|dirX|)
        val fovYdegApprox: Double  // 2*atan(max|dirY|)
    )

    companion object {
        private const val TAG = "ToF"
    }
}