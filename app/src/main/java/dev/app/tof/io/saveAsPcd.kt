package dev.app.tof.io

import android.content.Context
import android.util.Log
import dev.app.tof.core.Debug3DPoint
import java.io.File

private fun saveAsPcd(context: Context, points: List<Debug3DPoint>) {
    // 存在 app 自己的目錄
    val dir = context.getExternalFilesDir(null) ?: context.filesDir
    val outFile = File(dir, "tof_android.pcd")

    outFile.printWriter().use { pw ->
        pw.println("# .PCD v0.7 - Point Cloud Data file format")
        pw.println("VERSION 0.7")
        pw.println("FIELDS x y z")
        pw.println("SIZE 4 4 4")
        pw.println("TYPE F F F")
        pw.println("COUNT 1 1 1")
        pw.println("WIDTH ${points.size}")
        pw.println("HEIGHT 1")
        pw.println("VIEWPOINT 0 0 0 1 0 0 0")
        pw.println("POINTS ${points.size}")
        pw.println("DATA ascii")

        points.forEach { p ->
            // 你的 pcd 是 mm，還要把 x,y 反號回去
            val xMm = (-p.x) * 1000f
            val yMm = (-p.y) * 1000f
            val zMm = (p.z) * 1000f
            pw.println("$xMm $yMm $zMm")
        }
    }

    Log.d("ToF", "PCD saved: ${outFile.absolutePath}")
}
