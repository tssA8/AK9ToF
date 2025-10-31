package dev.app.tof

interface ToFSource {
    fun nextFrame(): ToFFrame?   // 沒有就回 null
}
