package dev.app.tof

import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.tooling.preview.Preview
import dev.app.tof.ui.theme.TofTheme
import kotlinx.coroutines.delay
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
    val processor = rememberToFProcessor()

    LaunchedEffect(Unit) {
        processor.start()
        while (isActive) {
            val frame = source.nextFrame() ?: break
            processor.submit(frame)
            delay(50)
        }
    }

    DisposableEffect(Unit) {
        onDispose { processor.stop() }
    }

    Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
        Greeting(
            name = "Android ToF",
            modifier = Modifier.padding(innerPadding)
        )
    }
} // ← 只需要這一個括號結束 ToFScreen。不要再多一個。


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
