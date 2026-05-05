// PolarOverlay.cs (SAFE + THICK + VIEW-ALIGNED)
// - No destruye hijos del robot aunque lo pegues en base_link.
// - Patch (cuadrado) naranja, anillos: azul/amarillo/rojo, sectores blancos suaves.
// - Grosor configurable por separado: patchWidth/ringWidth/sectorWidth.
// - LineAlignment.View para que se vea bien desde la cámara.

using System.Collections.Generic;
using UnityEngine;

public class PolarOverlay : MonoBehaviour
{
    [Header("Follow")]
    public Transform target;          // base_link del robot
    public float yOffset = 0.15f;     // levanta overlay para evitar z-fighting

    [Header("Square (egocentric patch)")]
    public float squareSize = 21f;    // 21x21 m (paper)

    [Header("Polar grid (paper defaults)")]
    public float innerRadius = 0.0f;            // R0 (0 si quieres [0,4), 0.8 si excluyes footprint)
    public float[] ringRadii = { 4f, 8f, 10f }; // R1,R2,R3
    public int sectors = 24;                    // K
    public int circleSegments = 160;

    [Header("Draw toggles")]
    public bool drawSquare = true;
    public bool drawRings = true;
    public bool drawSectorLines = true;

    [Header("Material")]
    public Material lineMaterial;     // si es null, crea uno simple

    [Header("Widths (thicker = easier to see)")]
    public float patchWidth  = 0.15f; // cuadrado
    public float ringWidth   = 0.12f; // anillos
    public float sectorWidth = 0.07f; // radios/sectores

    [Header("Colors")]
    public Color patchColor  = new Color(1f, 0.55f, 0f, 0.9f);   // naranja
    public Color sectorColor = new Color(1f, 1f, 1f, 0.45f);     // blanco suave
    public Color[] ringColors = new Color[]
    {
        new Color(0.2f, 0.55f, 1f, 0.9f),  // near: azul
        new Color(1f, 0.92f, 0.2f, 0.9f),  // mid: amarillo
        new Color(1f, 0.2f, 0.2f, 0.9f)    // far: rojo
    };

    [System.Serializable]
    public struct HighlightCell { public int ring; public int sector; }

    [Header("Optional highlights (wedge cells)")]
    public HighlightCell[] highlightedCells;
    public float highlightWidth = 0.18f;
    public Color highlightColor = new Color(1f, 0.1f, 0.1f, 0.95f);

    [Header("Render settings")]
    public bool viewAligned = true;   // LineAlignment.View (recomendado)
    public bool forceOverlayQueue = false; // si quieres empujar renderQueue (según pipeline)

    // Internals
    Transform overlayRoot;            // contenedor propio (seguro)
    LineRenderer squareLR;
    LineRenderer[] ringLRs;
    LineRenderer[] sectorLRs;
    LineRenderer[] highlightLRs;

    void Awake()
    {
        if (lineMaterial == null)
        {
            var shader = Shader.Find("Sprites/Default"); // simple y compatible
            lineMaterial = new Material(shader);
        }

        if (forceOverlayQueue && lineMaterial != null)
        {
            // Empuja a dibujarse "tarde". No garantiza siempre encima del terreno, pero ayuda.
            lineMaterial.renderQueue = 4000;
        }

        // Crea contenedor propio (no toca jerarquía del robot)
        var go = new GameObject("PolarOverlayRoot");
        overlayRoot = go.transform;

        RebuildAll();
    }

    void OnDestroy()
    {
        if (overlayRoot != null)
        {
            if (Application.isPlaying) Destroy(overlayRoot.gameObject);
            else DestroyImmediate(overlayRoot.gameObject);
        }
    }

    void LateUpdate()
    {
        if (!target || overlayRoot == null) return;

        // Posición y yaw del robot (sin modificar base_link)
        overlayRoot.position = target.position + Vector3.up * yOffset;
        overlayRoot.rotation = Quaternion.Euler(0f, target.eulerAngles.y, 0f);
    }

    void OnValidate()
    {
        sectors = Mathf.Max(1, sectors);
        circleSegments = Mathf.Max(16, circleSegments);
        squareSize = Mathf.Max(0.01f, squareSize);
        innerRadius = Mathf.Max(0f, innerRadius);

        patchWidth  = Mathf.Max(0.001f, patchWidth);
        ringWidth   = Mathf.Max(0.001f, ringWidth);
        sectorWidth = Mathf.Max(0.001f, sectorWidth);
        highlightWidth = Mathf.Max(0.001f, highlightWidth);

        if (ringRadii != null)
        {
            for (int i = 0; i < ringRadii.Length; i++)
                ringRadii[i] = Mathf.Max(innerRadius, ringRadii[i]);
        }
    }

    // Llama a esto si cambias parámetros en runtime y quieres reconstruir
    public void RebuildAll()
    {
        if (overlayRoot == null) return;

        ClearOverlayChildren();

        if (drawSquare) BuildSquare();
        if (drawRings) BuildRings();
        if (drawSectorLines) BuildSectors();
        BuildHighlights();
    }

    void ClearOverlayChildren()
    {
        for (int i = overlayRoot.childCount - 1; i >= 0; i--)
        {
            var c = overlayRoot.GetChild(i);
            if (Application.isPlaying) Destroy(c.gameObject);
            else DestroyImmediate(c.gameObject);
        }

        squareLR = null;
        ringLRs = null;
        sectorLRs = null;
        highlightLRs = null;
    }

    LineRenderer CreateLine(string name, float width, Color color, bool loop)
    {
        var go = new GameObject(name);
        go.transform.SetParent(overlayRoot, false);

        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = false;
        lr.loop = loop;
        lr.material = lineMaterial;
        lr.startWidth = width;
        lr.endWidth = width;
        lr.startColor = color;
        lr.endColor = color;

        // Para que siempre se vea bien desde cámara (muy útil en vista top-down)
        if (viewAligned) lr.alignment = LineAlignment.View;

        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;

        // Suavizado visual
        lr.numCapVertices = 6;
        lr.numCornerVertices = 6;

        return lr;
    }

    // -------- Geometry builders --------
    void BuildSquare()
    {
        squareLR = CreateLine("SquarePatch", patchWidth, patchColor, true);

        float h = squareSize * 0.5f;
        Vector3[] p =
        {
            new Vector3(-h, 0f, -h),
            new Vector3(-h, 0f,  h),
            new Vector3( h, 0f,  h),
            new Vector3( h, 0f, -h),
        };

        squareLR.positionCount = p.Length;
        squareLR.SetPositions(p);
    }

    void BuildRings()
    {
        if (ringRadii == null || ringRadii.Length == 0) return;

        ringLRs = new LineRenderer[ringRadii.Length];
        for (int r = 0; r < ringRadii.Length; r++)
        {
            Color c = (ringColors != null && ringColors.Length > r) ? ringColors[r] : Color.white;
            var lr = CreateLine($"Ring_{r}", ringWidth, c, true);
            ringLRs[r] = lr;

            float radius = ringRadii[r];
            Vector3[] pts = new Vector3[circleSegments];

            for (int i = 0; i < circleSegments; i++)
            {
                float t = (float)i / circleSegments * Mathf.PI * 2f;
                pts[i] = new Vector3(Mathf.Cos(t) * radius, 0f, Mathf.Sin(t) * radius);
            }

            lr.positionCount = pts.Length;
            lr.SetPositions(pts);
        }
    }

    void BuildSectors()
    {
        float outer = GetOuterRadius();

        sectorLRs = new LineRenderer[sectors];
        for (int s = 0; s < sectors; s++)
        {
            var lr = CreateLine($"SectorLine_{s}", sectorWidth, sectorColor, false);
            sectorLRs[s] = lr;

            float ang = (float)s / sectors * Mathf.PI * 2f;

            Vector3 a = new Vector3(Mathf.Cos(ang) * innerRadius, 0f, Mathf.Sin(ang) * innerRadius);
            Vector3 b = new Vector3(Mathf.Cos(ang) * outer,       0f, Mathf.Sin(ang) * outer);

            lr.positionCount = 2;
            lr.SetPositions(new Vector3[] { a, b });
        }
    }

    void BuildHighlights()
    {
        if (highlightedCells == null || highlightedCells.Length == 0) return;
        if (ringRadii == null || ringRadii.Length == 0) return;

        // Boundaries: [innerRadius, R1, R2, ...]
        float[] bounds = new float[ringRadii.Length + 1];
        bounds[0] = innerRadius;
        for (int i = 0; i < ringRadii.Length; i++) bounds[i + 1] = ringRadii[i];

        highlightLRs = new LineRenderer[highlightedCells.Length];

        for (int i = 0; i < highlightedCells.Length; i++)
        {
            int ring = Mathf.Clamp(highlightedCells[i].ring, 0, ringRadii.Length - 1);
            int sec  = Mathf.Clamp(highlightedCells[i].sector, 0, sectors - 1);

            float rIn  = bounds[ring];
            float rOut = bounds[ring + 1];

            float a0 = (float)sec / sectors * Mathf.PI * 2f;
            float a1 = (float)(sec + 1) / sectors * Mathf.PI * 2f;

            var lr = CreateLine($"Highlight_{i}_r{ring}_s{sec}", highlightWidth, highlightColor, true);
            highlightLRs[i] = lr;

            int arcN = Mathf.Max(10, circleSegments / sectors);
            var pts = new List<Vector3>(2 * (arcN + 1));

            // outer arc a0 -> a1
            for (int k = 0; k <= arcN; k++)
            {
                float t = Mathf.Lerp(a0, a1, (float)k / arcN);
                pts.Add(new Vector3(Mathf.Cos(t) * rOut, 0f, Mathf.Sin(t) * rOut));
            }
            // inner arc a1 -> a0
            for (int k = 0; k <= arcN; k++)
            {
                float t = Mathf.Lerp(a1, a0, (float)k / arcN);
                pts.Add(new Vector3(Mathf.Cos(t) * rIn, 0f, Mathf.Sin(t) * rIn));
            }

            lr.positionCount = pts.Count;
            lr.SetPositions(pts.ToArray());
        }
    }

    float GetOuterRadius()
    {
        if (ringRadii != null && ringRadii.Length > 0) return ringRadii[ringRadii.Length - 1];
        return squareSize * 0.5f;
    }
}