using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

[DisallowMultipleComponent]
public class AutoRecoveryURDF : MonoBehaviour
{
    [Header("URDF / Articulation")]
    public ArticulationBody rootAb;         // Asigna el ArticulationBody RAÍZ (isRoot=true)

    [Header("Respawn")]
    public Transform spawnPoint;            // Si es null, usa pose inicial
    public float cooldownAfterReset = 1f;   // s de “gracia” tras reset

    [Header("Detección de estado")]
    [Tooltip("Vuelco si la inclinación supera este ángulo respecto a +Y mundial")]
    public float maxTiltDeg = 75f;
    [Tooltip("Vuelco si el componente Y del vector up es inferior a esto")]
    public float minUpY = 0.15f;
    [Tooltip("Se considera atascado si la velocidad < stuckSpeed durante stuckSeconds")]
    public float stuckSpeed = 0.05f;
    public float stuckSeconds = 3f;
    [Tooltip("Se considera fuera si Y < minY")]
    public float minY = -5f;
    public Vector3 minBounds = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
    public Vector3 maxBounds = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);

    [Header("Detección de suelo / barrancos")]
    public LayerMask groundLayers = ~0;     // capas del terreno
    public float groundProbeDistance = 100f;
    [Tooltip("Si el suelo detectado está más lejos que esto => barranco")]
    public float cliffDropThreshold = 1.0f;

    [Header("Debug")]
    public bool drawDebug = true;

    // --- ROS topics (cámbialos si quieres) ---
    const string TOPIC_RESET = "/sim/reset";             // std_msgs/Empty (ROS -> Unity)
    const string TOPIC_DONE  = "/env/episode_done";      // std_msgs/String (Unity -> ROS)
    const string TOPIC_SUCC  = "/env/success";           // std_msgs/Empty (ROS -> Unity)

    // --- internos ---
    ArticulationBody[] _allAbs;
    Vector3 _startPos;
    Quaternion _startRot;
    float _stuckTimer, _cooldownUntil;

    ROSConnection _ros;

    void Awake()
    {
        _startPos = transform.position;
        _startRot = transform.rotation;
        _allAbs = GetComponentsInChildren<ArticulationBody>(true);

        // Si no asignaste rootAb, intenta hallar el raíz (isRoot)
        if (rootAb == null)
        {
            foreach (var ab in _allAbs) if (ab.isRoot) { rootAb = ab; break; }
            if (rootAb == null) rootAb = GetComponent<ArticulationBody>();
        }

        _ros = ROSConnection.instance;
        if (_ros != null)
        {
            _ros.Subscribe<EmptyMsg>(TOPIC_RESET, _ => ResetRobot(true));
            _ros.Subscribe<EmptyMsg>(TOPIC_SUCC,  _ => { ReportDone("SUCCESS"); ResetRobot(false); });
            _ros.RegisterPublisher<StringMsg>(TOPIC_DONE);
        }
        else
        {
            Debug.LogWarning("[AutoRecoveryURDF] No hay ROSConnection.instance en la escena.");
        }
    }

    void Update()
    {
        if (Time.time < _cooldownUntil) return;

        // Reset manual local (solo dev)
        if (Input.GetKeyDown(KeyCode.R)) { ReportDone("MANUAL"); ResetRobot(true); }

        // Checks “duros”: resetean y reportan FIN de episodio
        if (IsFlipped())           { ReportDone("FLIPPED");        ResetRobot(true);  return; }
        if (IsOutOfBounds())       { ReportDone("OUT_OF_BOUNDS");  ResetRobot(true);  return; }
        if (IsVoidOrCliffBelow())  { ReportDone("CLIFF");          ResetRobot(true);  return; }

        // Check “blando”: enderezar/respawn suave si se queda atascado
        if (IsStuck())             { ReportDone("STUCK");          ResetRobot(false); return; }
    }

    // --- Detecciones ---------------------------------------------------------

    bool IsVoidOrCliffBelow()
    {
        Vector3 origin = transform.position + Vector3.up * 0.2f; // un pelín arriba
        if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, groundProbeDistance, groundLayers, QueryTriggerInteraction.Ignore))
        {
            if (drawDebug) Debug.DrawLine(origin, hit.point, Color.cyan, 0.1f);
            return hit.distance > cliffDropThreshold; // suelo demasiado lejos => “barranco”
        }
        if (drawDebug) Debug.DrawLine(origin, origin + Vector3.down * groundProbeDistance, Color.red, 0.1f);
        return true; // ningún impacto => vacío/fin de mapa
    }

    bool IsFlipped()
    {
        float upDot = Vector3.Dot(transform.up, Vector3.up);
        float tiltDeg = Mathf.Acos(Mathf.Clamp(upDot, -1f, 1f)) * Mathf.Rad2Deg;
        return tiltDeg > maxTiltDeg || transform.up.y < minUpY;
    }

    bool IsOutOfBounds()
    {
        var p = transform.position;
        if (p.y < minY) return true;
        return p.x < minBounds.x || p.y < minBounds.y || p.z < minBounds.z ||
               p.x > maxBounds.x || p.y > maxBounds.y || p.z > maxBounds.z;
    }

    bool IsStuck()
    {
        float speed = rootAb ? rootAb.velocity.magnitude : 0f;
        if (speed < stuckSpeed)
        {
            _stuckTimer += Time.deltaTime;
            if (_stuckTimer >= stuckSeconds) { _stuckTimer = 0f; return true; }
        }
        else _stuckTimer = 0f;
        return false;
    }

    // --- Reset + reporte -----------------------------------------------------

    public void ResetRobot(bool hardReset)
    {
        Vector3 pos = spawnPoint ? spawnPoint.position : _startPos;
        Quaternion rot = spawnPoint ? spawnPoint.rotation : _startRot;

        // Congelar y poner a cero velocidades para evitar impulsos raros
        foreach (var ab in _allAbs)
        {
            ab.velocity = Vector3.zero;
            ab.angularVelocity = Vector3.zero;
            ab.immovable = true;
        }

        if (!hardReset && Vector3.Distance(transform.position, pos) < 2f)
        {
            // Enderezar en sitio (conserva heading actual en Y)
            pos = transform.position;
            rot = Quaternion.Euler(0f, transform.rotation.eulerAngles.y, 0f);
        }

        if (rootAb && rootAb.isRoot) rootAb.TeleportRoot(pos, rot);
        else { transform.SetPositionAndRotation(pos, rot); Physics.SyncTransforms(); }

        foreach (var ab in _allAbs) ab.immovable = false;

        _cooldownUntil = Time.time + cooldownAfterReset;
    }

    void ReportDone(string reason)
    {
        if (_ros == null) return;
        _ros.Publish(TOPIC_DONE, new StringMsg(reason));
    }

    // --- Gizmos --------------------------------------------------------------

    void OnDrawGizmosSelected()
    {
        if (!drawDebug) return;

        // Bounds
        if (float.IsFinite(minBounds.x) && float.IsFinite(maxBounds.x) &&
            float.IsFinite(minBounds.y) && float.IsFinite(maxBounds.y) &&
            float.IsFinite(minBounds.z) && float.IsFinite(maxBounds.z))
        {
            Gizmos.color = Color.yellow;
            Vector3 center = (minBounds + maxBounds) * 0.5f;
            Vector3 size = (maxBounds - minBounds);
            Gizmos.DrawWireCube(center, size);
        }

        // Raycast de suelo
        Gizmos.color = Color.cyan;
        Vector3 origin = Application.isPlaying ? transform.position + Vector3.up * 0.2f
                                               : transform.position + Vector3.up * 0.2f;
        Gizmos.DrawLine(origin, origin + Vector3.down * groundProbeDistance);
    }
}
