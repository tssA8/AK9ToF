package dev.app.tof

import java.util.concurrent.ArrayBlockingQueue

data class CalibrationResult(
    val valid: Boolean,
    val pointsCount: Int,
    val samplePoints: List<FloatArray> = emptyList()
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
                // 這裡如果 stop() 有 interrupt，take() 會丟 InterruptedException
                val frame = try {
                    queue.take()
                } catch (ie: InterruptedException) {
                    break
                }
                val result = processFrame(frame)
                if (result.valid) {
                    listener(result)
                }
            }
        }.apply { start() }
    }

    fun stop() {
        running = false
        // 把 queue 卡住的 take() 喚醒
        worker?.interrupt()
        worker = null
    }

    fun submit(frame: ToFFrame) {
        // 滿了就丟掉一個舊的，避免一直塞
        if (!queue.offer(frame)) {
            queue.poll()
            queue.offer(frame)
        }
    }

    private fun processFrame(frame: ToFFrame): CalibrationResult {
        val mask = buildValidMask(frame)
        val points = depthAmpTo3D(frame, mask)

        // 取前 5 個當 sample
        val sample = if (points.size > 5) points.subList(0, 5) else points

        return CalibrationResult(
            valid = points.isNotEmpty(),
            pointsCount = points.size,
            samplePoints = sample
        )
    }

    private fun buildValidMask(frame: ToFFrame): BooleanArray {
        val total = frame.width * frame.height
        val mask = BooleanArray(total)
        for (i in 0 until total) {
            val d = frame.depth[i].toInt()
            val a = frame.amp[i].toInt()
            mask[i] = d != 0 && a != 0 && a < 65000
        }
        return mask
    }

    private fun depthAmpTo3D(frame: ToFFrame, mask: BooleanArray): List<FloatArray> {
        val w = frame.width
        val h = frame.height
        val fx = ToFIntrinsics.FX
        val fy = ToFIntrinsics.FY
        val cx = ToFIntrinsics.CX
        val cy = ToFIntrinsics.CY

        val out = ArrayList<FloatArray>()
        for (y in 0 until h) {
            for (x in 0 until w) {
                val idx = y * w + x
                if (!mask[idx]) continue
                val dmm = frame.depth[idx].toInt()
                val z = dmm / 1000f
                val X = (x - cx) * z / fx
                val Y = (y - cy) * z / fy
                out.add(floatArrayOf(X, Y, z))
            }
        }
        return out
    }
}
