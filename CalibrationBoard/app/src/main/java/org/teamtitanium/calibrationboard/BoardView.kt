package org.teamtitanium.calibrationboard

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Typeface
import android.os.Handler
import android.os.Looper
import android.view.View
import androidx.core.graphics.withScale
import androidx.core.graphics.withTranslation
import org.json.JSONObject

class BoardView(context: Context) : View(context) {

    // ── Board constants (mirrors script.js) ────────────────────────────────
    private val ROWS = 8
    private val COLUMNS = 12
    private val MARKER_UNIT_WIDTH = 7
    private val MARKER_UNIT_HEIGHT = 7
    private val SQUARE_UNIT_WIDTH = MARKER_UNIT_WIDTH + 2   // 9

    // Physical DPI from the display — 1 raw pixel == 1/xdpi inches at true scale.
    // fitScale maps raw pixels onto screen px; actual size = fitScale * rawPx / xdpi inches.
    private val xdpi: Float = resources.displayMetrics.xdpi

    private val squareSizeRaw: Float = SQUARE_UNIT_WIDTH.toFloat()
    private val markerSizeRaw: Float = MARKER_UNIT_WIDTH.toFloat()

    private val boardWidthRaw: Float = COLUMNS * squareSizeRaw
    private val boardHeightRaw: Float = ROWS * squareSizeRaw

    // ── State ───────────────────────────────────────────────────────────────
    private var markerDictionary: JSONObject? = null
    private var fitScale: Float = 1f
    private var fillColor: Int = Color.BLACK
    private var boardOffsetX: Float = 0f
    private var boardOffsetY: Float = 0f

    // Real-world sizes (inches) computed after fitScale is known
    private var squareSizeIn: Float = 0f
    private var markerSizeIn: Float = 0f

    // ── Paint ───────────────────────────────────────────────────────────────
    private val backgroundPaint = Paint().apply {
        style = Paint.Style.FILL
        color = Color.WHITE
    }

    private val boardPaint = Paint().apply {
        isAntiAlias = false
        style = Paint.Style.FILL
        color = Color.BLACK
    }

    private val textPaint = Paint().apply {
        isAntiAlias = true
        style = Paint.Style.FILL
        color = Color.BLACK
        typeface = Typeface.MONOSPACE
    }


    // ── Load dictionary on a background thread ──────────────────────────────
    init {
        Thread {
            val json = context.assets.open("aruco_5x5_1000.json")
                .bufferedReader()
                .use { it.readText() }
            val parsed = JSONObject(json)
            Handler(Looper.getMainLooper()).post {
                markerDictionary = parsed
                invalidate()
            }
        }.start()
    }

    // ── Layout ───────────────────────────────────────────────────────────────
    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        recalculate(w, h)
    }

    private fun recalculate(w: Int, h: Int) {
        // Reserve space at the bottom for the text overlay
        textPaint.textSize = w * 0.018f  // ~1.8% of view width, scales with screen
        val textAreaHeight = textPaint.textSize * 2.5f

        val availableHeight = h - textAreaHeight

        val scaleX = w / boardWidthRaw
        val scaleY = availableHeight / boardHeightRaw
        fitScale = minOf(scaleX, scaleY)

        // Centre the board within the available area (above the text strip)
        boardOffsetX = (w - boardWidthRaw * fitScale) / 2f
        boardOffsetY = (availableHeight - boardHeightRaw * fitScale) / 2f

        // Black = true scale or larger; Red = board was shrunk to fit
        fillColor = if (fitScale >= 1f) Color.BLACK else Color.RED
        boardPaint.color = fillColor
        textPaint.color = fillColor

        // Physical sizes: each raw pixel is (1/xdpi) inches, scaled by fitScale
        squareSizeIn = squareSizeRaw * fitScale / xdpi
        markerSizeIn = markerSizeRaw * fitScale / xdpi
    }

    // ── Draw ─────────────────────────────────────────────────────────────────
    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        // Fill the entire canvas white so undrawn squares aren't the theme background colour
        canvas.drawColor(Color.WHITE)

        val dict = markerDictionary ?: return

        // Draw board centred on screen
        canvas.withTranslation(boardOffsetX, boardOffsetY) {
            withScale(fitScale, fitScale) {
                drawBoard(this, dict)
            }
        }

        // Draw text overlay below the board (unscaled coordinates)
        drawOverlay(canvas)
    }

    private fun drawBoard(canvas: Canvas, dict: JSONObject) {
        for (row in 0 until ROWS) {
            for (col in 0 until COLUMNS) {
                var count = col
                if (row % 2 == 1) count += 1

                if (count % 2 == 0) {
                    // Solid black square
                    canvas.drawRect(
                        col * squareSizeRaw,
                        row * squareSizeRaw,
                        (col + 1) * squareSizeRaw,
                        (row + 1) * squareSizeRaw,
                        boardPaint
                    )
                } else {
                    // ArUco marker square
                    val markerId = ((row * COLUMNS) + col) / 2
                    val bits = getMarkerBits(dict, markerId)
                    val margin = (squareSizeRaw - markerSizeRaw) / 2f
                    drawMarker(
                        canvas,
                        bits,
                        col * squareSizeRaw + margin,
                        row * squareSizeRaw + margin,
                        markerSizeRaw
                    )
                }
            }
        }
    }

    private fun drawMarker(canvas: Canvas, bits: List<Int>, x: Float, y: Float, size: Float) {
        val pixelW = size / MARKER_UNIT_WIDTH
        val pixelH = size / MARKER_UNIT_HEIGHT

        // Border ring
        for (i in 0 until MARKER_UNIT_HEIGHT) {
            for (j in 0 until MARKER_UNIT_WIDTH) {
                if (i == 0 || i == MARKER_UNIT_HEIGHT - 1 || j == 0 || j == MARKER_UNIT_WIDTH - 1) {
                    canvas.drawRect(
                        x + pixelW * j,
                        y + pixelH * i,
                        x + pixelW * j + pixelW,
                        y + pixelH * i + pixelH,
                        boardPaint
                    )
                }
            }
        }

        // Inner data bits (0 = black, 1 = white/leave empty)
        for (i in 0 until MARKER_UNIT_HEIGHT - 2) {
            for (j in 0 until MARKER_UNIT_WIDTH - 2) {
                val white = bits[i * (MARKER_UNIT_HEIGHT - 2) + j]
                if (white == 0) {
                    canvas.drawRect(
                        x + pixelW * (j + 1),
                        y + pixelH * (i + 1),
                        x + pixelW * (j + 1) + pixelW,
                        y + pixelH * (i + 1) + pixelH,
                        boardPaint
                    )
                }
            }
        }
    }

    private fun getMarkerBits(dict: JSONObject, id: Int): List<Int> {
        val bits = mutableListOf<Int>()
        val dataArray = dict.getJSONArray("data")
        val markerBytes = dataArray.getJSONArray(id)
        val totalBits = dict.getInt("width") * dict.getInt("height")

        for (byteIdx in 0 until markerBytes.length()) {
            val byte = markerBytes.getInt(byteIdx)
            val start = totalBits - bits.size
            val topBit = minOf(7, start - 1)
            for (i in topBit downTo 0) {
                bits.add((byte shr i) and 1)
            }
        }

        return bits
    }

    private fun drawOverlay(canvas: Canvas) {
        val boardBottomPx = boardOffsetY + boardHeightRaw * fitScale

        // Format to 4 decimal places, trim trailing zeros
        fun Float.fmt(): String = "%.4f".format(this).trimEnd('0').trimEnd('.')

        val squareStr = squareSizeIn.fmt()
        val markerStr = markerSizeIn.fmt()
        val dpiStr = "%.1f".format(xdpi)
        val text = "$COLUMNS x $ROWS  |  Square: ${squareStr}\"  |  Marker: ${markerStr}\"  |  DPI: $dpiStr"

        val textSize = textPaint.textSize
        val padding = textSize * 0.5f
        val textWidth = textPaint.measureText(text)

        // Background pill behind text
        val bgLeft = (width - textWidth) / 2f - padding
        val bgTop = boardBottomPx + padding
        val bgRight = (width + textWidth) / 2f + padding
        val bgBottom = boardBottomPx + textSize + padding * 2.5f

        canvas.drawRect(bgLeft, bgTop, bgRight, bgBottom, backgroundPaint)

        // Text centred horizontally, baseline inside the bg rect
        canvas.drawText(
            text,
            (width - textWidth) / 2f,
            bgBottom - padding * 0.8f,
            textPaint
        )
    }
}




