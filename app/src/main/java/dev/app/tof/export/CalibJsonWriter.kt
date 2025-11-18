// build.gradle(Module)

package dev.app.tof.export

import android.content.Context
import dev.app.tof.Case
import dev.app.tof.core.CalibrationResult
import dev.app.tof.core.Intrinsics
import kotlinx.serialization.*
import kotlinx.serialization.json.*
import java.io.File
import java.time.Instant

@Serializable data class ResolutionJson(val width: Int, val height: Int)
@Serializable data class DistortionJson(
    val model: String,
    val coeffs: List<Double>            // [k1,k2,p1,p2,k3]
)

@Serializable data class IntrinsicsJson(
    val device_id: String,
    val resolution: ResolutionJson,
    val fx: Double, val fy: Double,
    val cx: Double, val cy: Double,
    val distortion: DistortionJson,
    val depth_unit: String = "mm",
    val axis_convention: String = "right-handed",
    val camera_axes: String = "X right, Y down, Z forward"
)


// 小工具：把你的 Intrinsics → JSON 欄位
private fun Intrinsics.toJson(deviceId: String = "ToF_A"): IntrinsicsJson {
    val coeffs = if (rectified) {
        listOf(0.0, 0.0, 0.0, 0.0, 0.0)
    } else {
        listOf(k1.toDouble(), k2.toDouble(), p1.toDouble(), p2.toDouble(), k3.toDouble())
    }
    return IntrinsicsJson(
        device_id = deviceId,
        resolution = ResolutionJson(width, height),
        fx = fx.toDouble(),
        fy = fy.toDouble(),
        cx = cx.toDouble(),
        cy = cy.toDouble(),
        distortion = DistortionJson(model = "brown-conrady", coeffs = coeffs)
    )
}

@Serializable data class PlaneFitJson(
    val n: List<Double>,          // [nx, ny, nz]
    val d_m: Double,              // d 以公尺
    val d_mm: Double,             // d 以毫米
    val rmse_mm: Double,
    val inliers: Int,
    val total: Int,
    val inliers_ratio: Double,
    val tilt_deg: Double,
    val pitch_deg: Double,
    val yaw_deg: Double
)

@Serializable data class SourceFilesJson(
    val depth_raw: String,
    val amp_raw: String,
    val pcd: String
)

@Serializable data class CaseMetadataJson(
    val label: String,            // "0cm" / "12.8cm"
    val units: String = "mm"
)

@Serializable data class CalibJson(
    val schema_version: String = "1.0",
    val created_utc: String = Instant.now().toString(),
    val device: String = "ToF_A",
    val intrinsics: IntrinsicsJson,
    val plane_fit: PlaneFitJson,
    val source_files: SourceFilesJson,
    val case: CaseMetadataJson
)

object CalibJsonWriter {
    private val json = Json { prettyPrint = true; encodeDefaults = true }

    fun write(
        context: Context,
        outFileName: String,
        intrinsics: Intrinsics,
        case: Case,
        result: CalibrationResult
    ): File {
        val p = result.plane ?: error("No plane in result")
        val cj = CalibJson(
            intrinsics = intrinsics.toJson(deviceId = "ToF_A"),
            plane_fit  = PlaneFitJson(
                n = listOf(p.nx.toDouble(), p.ny.toDouble(), p.nz.toDouble()),
                d_m = p.dMeters.toDouble(),
                d_mm = p.dMm.toDouble(),
                rmse_mm = p.rmseMm,
                inliers = p.inliers,
                total = p.total,
                inliers_ratio = p.inlierRatio,
                tilt_deg = p.tiltDeg,
                pitch_deg = p.pitchDeg,
                yaw_deg = p.yawDeg
            ),
            source_files = SourceFilesJson(
                depth_raw = resName(context, case.depthId),
                amp_raw   = resName(context, case.ampId),
                pcd       = resName(context, case.pcdId)
            ),
            case = CaseMetadataJson(label = case.label)
        )

        val dir = File(context.getExternalFilesDir(null), "calib_json").apply { mkdirs() }
        val out = File(dir, outFileName)
        out.writeText(json.encodeToString(cj))
        return out
    }

    private fun resName(ctx: Context, rawId: Int): String {
        return "res/raw/" + ctx.resources.getResourceEntryName(rawId)  // 例: res/raw/amp_1
    }
}
