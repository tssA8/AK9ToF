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
// 資料結構（保持你原本結構）
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

data class CalibrationResult(
    val valid: Boolean,
    val pointsCount: Int,
    val samplePoints: List<Debug3DPoint> = emptyList(),
    val pitchDeg: Double? = null,
    val yawDeg: Double? = null,
    val tiltDeg: Double? = null,
    val rmseMm: Double? = null,
    val inlierRatio: Double? = null
)


// ==============================
//  ToFProcessor（改：可選 cacheDir）
// ==============================
class ToFProcessor(
    private val listener: (CalibrationResult) -> Unit,
    private val cacheDir: File? = null   // ← 新增：給就做 LUT 快取；不給就只在記憶體建
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
            val byPercent = 0.005f * zMeters
            return max(0.008f, byPercent)
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
        Log.d(TAG, "plane n=(${n[0]}, ${n[1]}, ${n[2]}), d=$d, " +
                "tilt=${"%.2f".format(tiltDeg)}°, pitch=${"%.2f".format(pitchDeg)}°, yaw=${"%.2f".format(yawDeg)}°, " +
                "rmse=${"%.1f".format(rmseMm)}mm, inliers=${"%.1f".format(inlierRatio*100)}%, 3D points=${points.size}")
        if (sample.isNotEmpty()) {
            val p0 = sample[0]
            Log.d(TAG, "pt[0] pix=(${p0.u},${p0.v}) depth=${p0.depthMm}mm amp=${p0.amp} → X=${p0.x}, Y=${p0.y}, Z=${p0.z}")
        }

        return CalibrationResult(
            valid = true,
            pointsCount = points.size,
            samplePoints = sample,
            tiltDeg = tiltDeg,
            pitchDeg = pitchDeg,
            yawDeg = yawDeg,
            rmseMm = rmseMm.toDouble(),
            inlierRatio = inlierRatio.toDouble()
        )
    }

    // ==============================
    // 內參與 LUT 構建（支援可選快取）
    // ==============================
    private fun ensureIntrinsicsAndRays(w: Int, h: Int) {
        val needRebuild = (activeK == null || activeK!!.width != w || activeK!!.height != h)
        if (!needRebuild) return

        // 以 640x480 為母參數等比例推到當前尺寸；若 SDK 已整流，打開 forceRectified
        activeK = CalibRepo.deriveForSize(newW = w, newH = h /*, forceRectified = true*/)

        val dir = cacheDir  // ← 拷貝到區域變數，智慧轉型就能生效
        rays = if (dir != null) {
            if (!dir.exists()) dir.mkdirs()
            val cacheFile = LutBuilder.cacheFile(dir, activeK!!)
            Log.d("ToF", "LUT path = ${cacheFile.absolutePath}")
            LutBuilder.load(cacheFile, activeK!!) ?: LutBuilder.build(activeK!!).also {
                LutBuilder.save(cacheFile, it)
            }
        } else {
            RaysLUT(activeK!!)
        }
        printedIntrinsics = false
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

    private fun estimatePlane(points: List<Debug3DPoint>): Plane? {
        if (points.size < 3) return null

        var sumX = 0.0; var sumY = 0.0; var sumZ = 0.0
        var sumXX = 0.0; var sumYY = 0.0; var sumXY = 0.0
        var sumXZ = 0.0; var sumYZ = 0.0

        for (p in points) {
            val x = p.x.toDouble()
            val y = p.y.toDouble()
            val z = p.z.toDouble()
            sumX += x; sumY += y; sumZ += z
            sumXX += x*x; sumYY += y*y; sumXY += x*y
            sumXZ += x*z; sumYZ += y*z
        }

        val n = points.size.toDouble()
        val Sxx = sumXX; val Syy = sumYY; val Sxy = sumXY
        val Sxz = sumXZ; val Syz = sumYZ
        val det = Sxx * Syy - Sxy * Sxy
        if (abs(det) < 1e-8) {
            val zBar = sumZ / n
            return Plane(floatArrayOf(0f, 0f, 1f), (-zBar).toFloat())
        }

        val a = ( Syy * Sxz - Sxy * Syz ) / det
        val b = ( Sxx * Syz - Sxy * Sxz ) / det
        val xBar = sumX / n; val yBar = sumY / n; val zBar = sumZ / n
        val c = zBar - a * xBar - b * yBar   // z = a x + b y + c

        val nuX = a; val nuY = b; val nuZ = -1.0
        val scale = sqrt(nuX*nuX + nuY*nuY + nuZ*nuZ)
        val nx = (nuX / scale).toFloat()
        val ny = (nuY / scale).toFloat()
        val nz = (nuZ / scale).toFloat()
        val d = (c / scale).toFloat()
        return Plane(floatArrayOf(nx, ny, nz), d)
    }

    private fun estimatePlaneRansac(points: List<Debug3DPoint>, iters:Int=120): Plane? {
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
            val norm = sqrt(nx*nx + ny*ny + nz*nz)
            if (norm < 1e-6) return null
            nx /= norm; ny /= norm; nz /= norm
            var d = -(nx*p0.x + ny*p0.y + nz*p0.z)
            if (nz < 0f) { nx = -nx; ny = -ny; nz = -nz; d = -d }
            return Plane(floatArrayOf(nx, ny, nz), d)
        }

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
                val distM = abs(pl.normal[0]*p.x + pl.normal[1]*p.y + pl.normal[2]*p.z + pl.d)
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
            val distM = abs(bp.normal[0]*p.x + bp.normal[1]*p.y + bp.normal[2]*p.z + bp.d)
            if (distM <= max(0.008f, 0.005f * p.z)) inlierSet.add(p)
        }
        Log.d(TAG, "RANSAC inliers=${inlierSet.size}/${points.size}")

        return estimatePlane(inlierSet)
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

    companion object {
        private const val TAG = "ToF"
    }
}