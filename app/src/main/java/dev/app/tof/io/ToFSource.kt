package dev.app.tof.io

import dev.app.tof.core.ToFFrame

interface ToFSource {
    fun nextFrame(): ToFFrame?   // 沒有就回 null
}
