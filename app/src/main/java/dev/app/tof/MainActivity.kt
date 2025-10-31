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

    // 1) 資料來源：從 res/raw 讀剛剛放的 depth1.txt 跟 amp_1.txt
    val source: ToFSource = remember { RawToFSource(context) }

    // 2) 背景算 frame 的 processor
    val processor = rememberToFProcessor()

    // 3) 啟動 loop：讀一幀 → 丟給 processor
    LaunchedEffect(Unit) {
        processor.start()
        while (isActive) {
            val frame = source.nextFrame()
            if (frame != null) {
                processor.submit(frame)
            } else {
                // 沒有下一幀了就結束 loop
                break
            }
            delay(50) // 模擬 20fps
        }
    }

    // 4) 組件被銷毀時把 thread 收掉
    DisposableEffect(Unit) {
        onDispose {
            processor.stop()
        }
    }

    Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
        Greeting(
            name = "Android ToF",
            modifier = Modifier.padding(innerPadding)
        )
    }
}

@Composable
private fun rememberToFProcessor(): ToFProcessor {
    return remember {
        ToFProcessor { result ->
            if (result.valid) {
                Log.d("ToF", "3D points = ${result.pointsCount}")
                result.samplePoints.forEachIndexed { i, p ->
                    Log.d("ToF", "pt[$i] = x=${p[0]}, y=${p[1]}, z=${p[2]}")
                }
            } else {
                Log.d("ToF", "invalid frame")
            }
        }
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
