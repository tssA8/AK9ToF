package dev.app.tof

import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.tooling.preview.Preview
import dev.app.tof.ui.theme.TofTheme
import kotlinx.coroutines.isActive

class MainActivity : ComponentActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        setContent {
            TofTheme {
                ToFScreen()  // 不用再傳 folder 了，我們從 res/raw 讀
            }
        }
    }
}


data class Case(
    val label: String,       // "0cm" / "12.8cm"
    val depthId: Int,        // R.raw.depth1 / depth2
    val ampId: Int,          // R.raw.amp_1 / amp_2
    val pcdId: Int           // R.raw.pcd_1 / pcd_2
)

@Composable
fun ToFScreen() {
    val context = LocalContext.current

    // 兩個測試案例：1=0cm, 2=12.8cm
    val cases = remember {
        listOf(
            Case("0cm",     R.raw.depth1, R.raw.amp_1, R.raw.pcd_1),
            Case("12.8cm",  R.raw.depth2, R.raw.amp_2, R.raw.pcd_2)
        )
    }

    LaunchedEffect(Unit) {
        for (c in cases) {
            Log.d("ToF", "===== Running case: ${c.label} =====")

            // 每組案例各自建立 processor（乾淨狀態）
            val cacheDir = java.io.File(context.getExternalFilesDir(null), "raylut").apply { mkdirs() }
            val processor = ToFProcessor(
                listener = { result ->
                    if (result.valid) {
                        result.plane?.let { p ->
                            Log.d("ToF",
                                "n=(${p.nx},${p.ny},${p.nz}), d=${"%.6f".format(p.dMeters)} m (${p.dMm.toInt()} mm), " +
                                        "tilt=${"%.2f".format(p.tiltDeg)}°, pitch=${"%.2f".format(p.pitchDeg)}°, yaw=${"%.2f".format(p.yawDeg)}°, " +
                                        "rmse=${"%.1f".format(p.rmseMm)}mm, inliers=${p.inliers}/${p.total} (${p.inlierRatio*100}%)"
                            )
                        }
                    }
                },
                cacheDir = cacheDir
            )

            // 載入對應這組的參考 PCD
            try {
                val cloud = PcdReader.readAsciiFromRaw(context, c.pcdId)
                processor.updateSdkCloud(cloud.x, cloud.y, cloud.z) // 你的接口名稱照舊
                Log.d("ToF", "[${c.label}] Loaded PCD points = ${cloud.x.size}")
            } catch (t: Throwable) {
                Log.w("ToF","[${c.label}] PCD load failed: ${t.message}")
            }

            // 準備這組 frame 的資料來源
            val source: ToFSource = RawToFSource(
                ctx = context,
                depthRawId = c.depthId,
                ampRawId = c.ampId,
                width = 120, height = 90
            )

            // 跑單幀（若要模擬串流可多幀+delay）
            processor.start()
            source.nextFrame()?.let { processor.submit(it) }
            processor.stop()
        }

        Log.d("ToF", "===== All cases done =====")
    }
}


fun pickRaylutCacheDir(context: android.content.Context): java.io.File {
    // 所有外部檔案目錄（[0] 通常是內建外部儲存；[1] 若存在多半是可移除 SD 卡）
    val dirs = context.getExternalFilesDirs(null)
    val sdCandidate = dirs?.firstOrNull { it != null && android.os.Environment.isExternalStorageRemovable(it) }
    val base = sdCandidate ?: dirs?.firstOrNull() ?: context.filesDir  // 三段式 fallback
    return java.io.File(base, "raylut").apply { mkdirs() }
}


@Composable
private fun rememberToFProcessor(): ToFProcessor {
    val context = LocalContext.current
    return remember {
        val cacheDir = pickRaylutCacheDir(context)   // ← 換成 SD 卡 app 目錄（若有）
        ToFProcessor(
            listener = { result ->
                if (result.valid) {
                    Log.d("ToF", "3D points = ${result.pointsCount}")
                    result.samplePoints.forEachIndexed { i, p ->
                        Log.d("ToF",
                            "pt[$i] pix=(${p.u},${p.v}) depth=${p.depthMm}mm amp=${p.amp} → X=${p.x}, Y=${p.y}, Z=${p.z}"
                        )
                    }
                } else {
                    Log.d("ToF", "invalid frame")
                }
            },
            cacheDir = cacheDir
        )
    }
}


@Composable
fun Greeting(name: String, modifier: Modifier = Modifier) {
    Text(
        text = "Hello $name!",
        modifier = modifier
    )
}

@Preview(showBackground = true)
@Composable
fun GreetingPreview() {
    TofTheme {
        Greeting("Android")
    }
}
