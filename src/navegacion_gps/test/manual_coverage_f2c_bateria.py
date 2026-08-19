"""Bateria de casos de CAMPO con Fields2Cover, contra el backend real.

No es un test de pytest: necesita el route_executor y el coverage_server
corriendo, asi que se corre a mano. Se guarda aca porque es lo que hay que
pasar antes de dar por buena cualquier cosa que toque el planificador agricola.

    ros2 run ... route_executor  (coverage_planner=fields2cover)
    ros2 run opennav_coverage opennav_coverage  (configure + activate)
    python3 src/navegacion_gps/test/manual_coverage_f2c_bateria.py

Cada caso comprueba tres cosas:

  Rmin           el radio de curvatura de los giros no baja del minimo del
                 vehiculo. Un giro mas cerrado que eso no se puede ejecutar y
                 el robot se descarrila tratando de seguirlo.
  trabajo_fuera  ninguna pasada de trabajo corta fuera del lote.
  en_exclusion   ninguna pasada de trabajo corta dentro de una exclusion.

Los casos cubren la exclusion en el medio, pegada al borde y en una esquina,
que son los que rompian el rodeo del planificador propio.
"""
import math, sys, rclpy
from rclpy.node import Node
from interfaces.srv import GenerateCoveragePlanLL
from interfaces.msg import GeoRing, NoGoPoint

LAT0, LON0 = -31.485802, -64.241050
MLAT = 111320.0; MLON = 111320.0 * math.cos(math.radians(LAT0))
RMIN = 2.9
v = lambda x, y: NoGoPoint(lat=LAT0 + y / MLAT, lon=LON0 + x / MLON)
xy = lambda lat, lon: ((lon - LON0) * MLON, (lat - LAT0) * MLAT)

def ring(pts):
    r = GeoRing(); r.vertices = [v(x, y) for x, y in pts]; return r

def octogono(R=25.0, N=8):
    return [(R*math.cos(2*math.pi*i/N), R*math.sin(2*math.pi*i/N)) for i in range(N)]

def cuadro(cx, cy, w, h):
    return [(cx-w/2, cy-h/2), (cx+w/2, cy-h/2), (cx+w/2, cy+h/2), (cx-w/2, cy+h/2)]

def dentro(p, poly):
    x, y = p; inside = False; n = len(poly)
    for i in range(n):
        ax, ay = poly[i]; bx, by = poly[(i+1) % n]
        if (ay > y) != (by > y):
            if ax + ((y-ay)/(by-ay))*(bx-ax) > x: inside = not inside
    return inside

def dist_borde(p, poly):
    x, y = p; best = 1e9; n = len(poly)
    for i in range(n):
        ax, ay = poly[i]; bx, by = poly[(i+1) % n]
        dx, dy = bx-ax, by-ay; L2 = dx*dx+dy*dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/L2))
        best = min(best, math.hypot(x-(ax+t*dx), y-(ay+t*dy)))
    return best

def encoger(poly, f=0.85):
    cx = sum(p[0] for p in poly)/len(poly); cy = sum(p[1] for p in poly)/len(poly)
    return [(cx+(x-cx)*f, cy+(y-cy)*f) for x, y in poly]

def radio_min(pts, fases):
    peor, apretados = 1e9, 0
    for i in range(1, len(pts)-1):
        if not (fases[i-1] == fases[i] == fases[i+1] == "turn"): continue
        a, b, c = pts[i-1], pts[i], pts[i+1]
        l1, l2, l3 = math.dist(a,b), math.dist(b,c), math.dist(a,c)
        if min(l1, l2, l3) < 1e-6: continue
        area2 = abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
        if area2 < 1e-9: continue
        R = (l1*l2*l3)/(2*area2)
        peor = min(peor, R)
        if R < RMIN*0.9: apretados += 1
    return peor, apretados

rclpy.init(); n = Node("bateria")
c = n.create_client(GenerateCoveragePlanLL, "/route_executor/generate_coverage_plan_ll")
c.wait_for_service(timeout_sec=20.0)

def correr(nombre, poly, excls=(), corte=2.0):
    r = GenerateCoveragePlanLL.Request()
    r.start_lat, r.start_lon, r.start_yaw_deg = LAT0, LON0, 0.0
    r.field_length_m = r.field_width_m = 60.0
    r.cutter_width_m, r.overlap_ratio = corte, 0.15
    r.min_turning_radius_m, r.waypoint_spacing_m = RMIN, 2.0
    r.side = "left"
    r.coverage_polygon = ring(poly)
    r.coverage_exclusions = [ring(e) for e in excls]
    f = c.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=90.0)
    res = f.result()
    if res is None or not res.ok:
        print(f"  {nombre:32s} FALLA: {'timeout' if res is None else res.error[:52]}")
        return False
    pts = [xy(la, lo) for la, lo in zip(res.sampled_lats, res.sampled_lons)]
    fases = list(res.sampled_phases)
    R, ap = radio_min(pts, fases)
    fuera = max((dist_borde(p, poly) for p, fa in zip(pts, fases)
                 if fa == "row" and not dentro(p, poly)), default=0.0)
    en_excl = sum(1 for p, fa in zip(pts, fases) if fa == "row"
                  and any(dentro(p, encoger(e)) for e in excls))
    ok = ap == 0 and fuera < 0.05 and en_excl == 0
    print(f"  {nombre:32s} {'OK ' if ok else 'MAL'} pasadas={res.row_count:3d} "
          f"Rmin={R:5.2f}m apretados={ap:3d} trabajo_fuera={fuera:.2f}m "
          f"en_exclusion={en_excl}")
    return ok

print("=== lote sin exclusiones ===")
todo = [correr("octogono 50 m", octogono())]
todo.append(correr("octogono, corte 3 m", octogono(), corte=3.0))
todo.append(correr("octogono chico 30 m", octogono(R=15.0)))
print("=== exclusion EN EL MEDIO ===")
todo.append(correr("exclusion central 6x4", octogono(), [cuadro(0, 0, 6, 4)]))
todo.append(correr("exclusion central grande 14x10", octogono(), [cuadro(0, 0, 14, 10)]))
print("=== exclusion EN EL BORDE ===")
todo.append(correr("exclusion pegada al borde este", octogono(), [cuadro(20, 0, 6, 4)]))
todo.append(correr("exclusion pegada al borde norte", octogono(), [cuadro(0, 20, 6, 4)]))
todo.append(correr("exclusion en esquina NE", octogono(), [cuadro(14, 14, 5, 5)]))
print("=== varias exclusiones ===")
todo.append(correr("dos: medio + borde", octogono(),
                   [cuadro(0, 0, 6, 4), cuadro(18, -2, 5, 4)]))
todo.append(correr("tres exclusiones", octogono(),
                   [cuadro(-12, 0, 5, 4), cuadro(0, 12, 5, 4), cuadro(12, -8, 5, 4)]))
print(f"\nresultado: {sum(todo)}/{len(todo)} casos OK")
rclpy.shutdown()
sys.exit(0 if all(todo) else 1)
