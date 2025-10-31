package dev.app.tof

import android.util.Log
import java.util.concurrent.ArrayBlockingQueue

// 1) 為了 debug 新增的資料結構
data class Debug3DPoint(
    val u: Int,          // 原始像素 x
    val v: Int,          // 原始像素 y
    val depthMm: Int,    // 原始深度 (mm)
    val amp: Int,        // 原始強度
    val x: Float,        // 轉完的 3D X
    val y: Float,        // 轉完的 3D Y
    val z: Float         // 轉完的 3D Z (m)
)

// 2) 回呼的結果也要能帶 sample 出去
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
                } catch (ie: InterruptedException) {
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
            queue.poll()
            queue.offer(frame)
        }
    }

    private fun processFrame(frame: ToFFrame): CalibrationResult {
        // 1) 做 mask
        val mask = buildValidMask(frame)

        // 2) 真的做 2D → 3D，這裡就把 (u,v,depth,amp) 都塞進去
        val points = depthAmpTo3D(frame, mask)

        // 3) 只拿前幾個出來給 UI/Log 看
        val sample = if (points.size > 5) points.subList(0, 5) else points

        return CalibrationResult(
            valid = points.isNotEmpty(),
            pointsCount = points.size,
            samplePoints = sample
        )
    }

    // 判斷哪些點要丟掉
    private fun buildValidMask(frame: ToFFrame): BooleanArray {
        val total = frame.width * frame.height
        val mask = BooleanArray(total)
        for (i in 0 until total) {
            val d = frame.depth[i]   // Int
            val a = frame.amp[i]     // Int
            // 這裡可以照你剛剛的 tof 規則調
            mask[i] = d != 0 && a != 0 && a < 65000
        }
        return mask
    }

    // 真的把 2D pixel 轉 3D，順便把 pixel / depth / amp 保留下來
    private fun depthAmpTo3D(
        frame: ToFFrame,
        mask: BooleanArray
    ): List<Debug3DPoint> {
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

                val dmm = frame.depth[idx]   // mm
                val amp = frame.amp[idx]

                val z = dmm / 1000f          // m
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
