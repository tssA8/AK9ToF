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

@Composable
fun ToFScreen() {
    val context = LocalContext.current
    val source: ToFSource = remember { RawToFSource(context) }

    // 建議把 LUT 快取放到 app 的 externalFiles（若要存到 SDCard 你之前已加權限）
    val cacheDir = remember {
        // 改成 getExternalFilesDir 可讓你看到實體檔案 /Android/data/<pkg>/files/raylut
        java.io.File(context.getExternalFilesDir(null), "raylut")
    }
    val processor = remember {
        ToFProcessor(
            listener = { result ->
                if (result.valid) {
                    Log.d("ToF","3D points = ${result.pointsCount}")
                } else {
                    Log.d("ToF","invalid frame")
                }
            },
            cacheDir = cacheDir
        )
    }

    // 讀 PCD（只做一次），交給 processor 做 LUT 對照驗證
    LaunchedEffect(Unit) {
        try {
            val cloud = PcdReader.readAsciiFromRaw(context, R.raw.pcd_2) // 你放的檔名 pcd_1.pcd
            processor.updateSdkCloud(cloud.x, cloud.y, cloud.z)
            Log.d("ToF","Loaded PCD points = ${cloud.x.size}")
        } catch (t: Throwable) {
            Log.w("ToF","PCD load failed: ${t.message}")
        }

        processor.start()
        while (isActive) {
            val frame = source.nextFrame() ?: break
            processor.submit(frame)
            // 單幀測試即可；若要模擬串流，加 delay
            // delay(50)
        }
    }

    DisposableEffect(Unit) {
        onDispose { processor.stop() }
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
