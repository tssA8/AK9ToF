package dev.app.tof

import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.tooling.preview.Preview
import dev.app.tof.core.ToFProcessor
import dev.app.tof.export.CalibJsonWriter
import dev.app.tof.io.PcdReader
import dev.app.tof.io.RawToFSource
import dev.app.tof.io.ToFSource
import dev.app.tof.ui.theme.TofTheme

// ---------------------- MainActivity ----------------------

class MainActivity : ComponentActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        setContent {
            TofTheme {
                ToFScreen()
            }
        }
    }
}

/**
 * 一個測試案例：
 * - label: 只是用來命名 / log
 * - depthId / ampId / pcdId: 對應 res/raw 檔
 * - baselineMm: 相對第一顆 ToF (0cm / pcd_1) 在 X 方向的位移 (mm)
 * - sensorId: 使用哪一顆 ToF 的內參（SENSOR_1 / SENSOR_2）
 */
data class Case(
    val label: String,                     // "0cm" / "31.02cm"
    val depthId: Int,                     // R.raw.depth1 / depth2
    val ampId: Int,                       // R.raw.amp_1 / amp_2
    val pcdId: Int,                       // R.raw.pcd_1 / pcd_2
    val baselineMm: Double,               // 0.0 / 310.2
    val sensorId: ToFProcessor.ToFSensorId // 使用哪一顆 ToF
)

// ---------------------- UI / pipeline ----------------------

@Composable
fun ToFScreen() {
    val context = LocalContext.current

    // ⚠️ 目前先示範：兩個案例都用第一顆 ToF
    // 之後如果你有第二顆的 depth/amp/pcd，就再新增 Case，改成 SENSOR_2 即可
    val cases = remember {
        listOf(
            Case(
                label = "0cm",
                depthId = R.raw.depth1,
                ampId = R.raw.amp_1,
                pcdId = R.raw.pcd_1,
                baselineMm = 0.0,
                sensorId = ToFProcessor.ToFSensorId.SENSOR_1
            ),
            Case(
                label = "31.02cm",
                depthId = R.raw.depth2,
                ampId = R.raw.amp_2,
                pcdId = R.raw.pcd_2,
                baselineMm = 310.2,  // 31.02 cm
                sensorId = ToFProcessor.ToFSensorId.SENSOR_1
            )
        )
    }

    LaunchedEffect(Unit) {
        for (c in cases) {
            Log.d("ToF", "===== Running case: ${c.label} (sensor=${c.sensorId}) =====")

            val cacheDir = java.io.File(context.getExternalFilesDir(null), "raylut").apply { mkdirs() }

            // 先宣告，listener 會用到
            lateinit var processor: ToFProcessor

            processor = ToFProcessor(
                listener = { result ->
                    if (result.valid) {
                        result.plane?.let { p ->
                            Log.d(
                                "ToF",
                                "n=(${p.nx},${p.ny},${p.nz}), d=${"%.6f".format(p.dMeters)} m (${p.dMm.toInt()} mm), " +
                                        "tilt=${"%.2f".format(p.tiltDeg)}°, pitch=${"%.2f".format(p.pitchDeg)}°, yaw=${
                                            "%.2f".format(
                                                p.yawDeg
                                            )
                                        }°, " +
                                        "rmse=${"%.1f".format(p.rmseMm)}mm, inliers=${p.inliers}/${p.total} (${p.inlierRatio * 100}%)"
                            )
                        }

                        val file = CalibJsonWriter.write(
                            context = context,
                            outFileName = "tof_${c.sensorId.name.lowercase()}_${
                                c.label.replace(
                                    ' ',
                                    '_'
                                ).replace('.', '_')
                            }.json",
                            intrinsics = processor.getActiveIntrinsics(),
                            case = c,          // 這裡會把 baselineMm 一起寫進 JSON
                            result = result
                        )
                        Log.d("ToF", "[${c.label}] JSON saved: ${file.absolutePath}")
                    }
                },
                cacheDir = cacheDir,
                sensorId = c.sensorId   // ⭐ 這個最重要：告訴 Processor 用哪一顆 ToF 的內參
            )

            // 載入對應這組的 PCD（SDK 算出來的點雲）
            try {
                val cloud = PcdReader.readAsciiFromRaw(context, c.pcdId)
                processor.updateSdkCloud(cloud.x, cloud.y, cloud.z)
                Log.d("ToF", "[${c.label}] Loaded PCD points = ${cloud.x.size}")
            } catch (t: Throwable) {
                Log.w("ToF", "[${c.label}] PCD load failed: ${t.message}")
            }

            // 建立這組的資料來源（離線 TXT → frame）
            val source: ToFSource = RawToFSource(
                ctx = context,
                depthRawId = c.depthId,
                ampRawId = c.ampId,
                width = 120,
                height = 90
            )

            processor.start()
            source.nextFrame()?.let { processor.submit(it) }
            processor.stop()
        }

        Log.d("ToF", "===== All cases done =====")
    }
}

// 下面 rememberToFProcessor / Greeting 保留原來的就好
@Composable
fun Greeting(name: String, modifier: Modifier = Modifier) {
    Text(text = "Hello $name!", modifier = modifier)
}

@Preview(showBackground = true)
@Composable
fun GreetingPreview() {
    TofTheme { Greeting("Android") }
}
