"""
Корпус для электровелосипеда: аккумулятор 18650 + BMS + контроллер.
Параметрическая модель на CadQuery.

Концепция (по эскизам Виталика):
  * корпус -- трапеция в треугольнике рамы, подвешивается под верхнюю трубу
    на двух ремнях, ремни продеваются в пазы петель на боковых стенках;
  * эскиз: верх 340, низ 280, высота 160, ширина 100 мм -- ширина и высота
    пересчитываются по содержимому (см. отчёт);
  * сверху отсек АКБ: батарея 270x110x73 стоит НА БОКУ (73 мм поперёк
    велосипеда -- меньше ширина, больше места коленям), BMS 140x105x20
    остаётся прикрученной к её грани (как на фото);
  * съёмная полка-перегородка;
  * снизу отсек контроллера: контроллер 172x83x42 (с ушками 196) лежит
    плашмя, ушками на саморезах к поперечным опорам на дне (дно глухое);
  * крышка сверху, 4 откидные защёлки «как на ящике для инструментов»;
  * провода выходят назад (торец B, к сиденью) вверху, у самой трубы, и идут
    вдоль рамы;
  * на торцах A/B и на дне C -- рёбра 5 мм (по эскизу).

Оси: X -- вдоль верхней трубы (+X назад, к сиденью), Y -- поперёк велосипеда,
Z -- вверх.

Запуск:   python enclosure.py        -> файлы в ./out
Все размеры в миллиметрах. Меняйте только блок ПАРАМЕТРЫ.
"""

from __future__ import annotations

import math
import pathlib

import cadquery as cq

# =============================================================================
# ПАРАМЕТРЫ
# =============================================================================

# --- Аккумулятор (замер) ------------------------------------------------------
BAT_L, BAT_WIDE, BAT_THIN = 270.0, 110.0, 73.0
BAT_ON_SIDE = True     # True: тонкая сторона (73) поперёк велосипеда
# --- BMS (замер), прикручена на широкую грань батареи -------------------------
BMS_L, BMS_W, BMS_T = 140.0, 105.0, 20.0

# --- Контроллер (эскиз 3) ----------------------------------------------------
CTRL_L, CTRL_W, CTRL_H = 172.0, 83.0, 42.0   # корпус без ушек
CTRL_TAB_L = 12.0                            # ушко с каждого торца
CTRL_HOLE_SPAN = 55.0  # ! межцентровое отверстий в ушке -- УТОЧНИТЬ
CTRL_CABLE_SPACE = 45.0  # запас на изгиб проводов у торца B
MOUNT_H = 4.0          # высота поперечных опор под ушки
PILOT_D = 3.4          # пилотное отверстие под саморез 4.2 мм
PILOT_DEPTH = 6.0

# --- Корпус -------------------------------------------------------------------
TOP_L = 340.0          # длина по верху (эскиз)
BOT_L = 280.0          # длина по низу (эскиз)
W_MIN = 0.0            # можно задать минимальную ширину (эскиз: 100)
WALL = 3.0             # стенки и дно (PETG/ASA)
LID_T = 3.5
CLEAR = 2.0            # зазор до содержимого (на сторону)
R_BOTTOM = 14.0        # скругление низа по торцам (вид сбоку)
R_SIDE = 6.0           # скругление всех рёбер боковых стенок
R_LID = 1.2            # скругление верхней кромки крышки
SHELF_T = 3.0
LEDGE = 4.0            # уступы под полку
LIP_H = 4.0            # центрирующий буртик крышки
LIP_T = 2.0
LID_OVER = 1.5         # свес крышки по периметру

# --- Защёлки «лягушка», как на ящике для инструментов -----------------------
# Проушина на корпусе + рычаг + серьга, зацеп на крышке. Серьга проходит через
# «мёртвую точку»: закрытый замок держит сам, от тряски не открывается.
# Координаты ниже -- от наружной плоскости стенки (y) и от верха основания (z).
LATCH_X = (-70.0, 70.0)  # по две на каждой длинной стороне
LATCH_P1 = (7.0, -24.0)  # ось рычага: винт M4 x 20 с гайкой
LATCH_R = 9.0            # плечо рычага до оси серьги
LATCH_DELTA = 16.0       # заход за мёртвую точку, градусы (~0.35 мм «щелчка»)
BAIL_ARM_W = 7.0         # плечи серьги (ширина)
LATCH_KY = LID_OVER + BAIL_ARM_W / 2 + 0.5  # перекладина серьги от стенки
LATCH_PRELOAD = 0.2      # натяг серьги, мм (в печатной детали серьга короче)
LUG_HALF = 4.5           # полутолщина проушины на корпусе (9 мм)
CHEEK_T = 4.0            # щёки рычага
BAIL_T = 3.5             # плечи серьги (толщина)
BAIL_BAR_D = 4.5         # перекладина серьги
PIN_D = 4.0              # ось рычага (M4)
PEG_D = 3.8              # шипы серьги в отверстиях рычага (отверстия 4.1)
GAP = 0.3

# --- Рёбра на торцах и дне (эскиз 1, виды A/B и C) --------------------------
RIB_H = 5.0
RIB_BASE, RIB_TOP = 8.0, 4.0
RIB_GAP_AB = 35.0      # просвет между рёбрами на торцах A/B
RIB_EDGE_C = 20.0      # отступ рёбер дна от длинных кромок

# --- Ремни под верхнюю трубу -------------------------------------------------
STRAP_W, STRAP_T = 27.0, 4.5  # паз под стропу/липучку 25 мм
STRAP_X = 125.0               # положение ремней от центра (+/-)
LOOP_BAR = 3.0                # толщина перемычки петли

# --- Выводы: торец B (+X) -- задний, к сиденью -------------------------------
GLAND_M20 = 20.4       # жгут контроллера (вверху, у трубы)
GLAND_M16 = 16.4       # второй жгут / силовые
CHARGE_PORT = 16.2     # GX16 / разъём зарядки
VENT_M12 = 12.4        # мембранный клапан (торец A)
HOLE_CHAMFER = 0.8

# --- Разрезка под принтер Bambu Lab P1S (стол 256 x 256 x 256) ---------------
PRINT_BED = (256.0, 256.0, 256.0)
# Корпус, крышка и полка длиннее стола -> режутся пополам по x=0.
JOINT_FL_OUT = 9.0     # наружный фланец стыка корпуса (выступ от стенки)
JOINT_FL_T = 5.0       # толщина фланца у каждой половины
JOINT_BOLT_D = 3.4     # болты M3 x 14 + гайки
JOINT_BOLT_Z = (16.0, 87.0, 158.0)  # высоты болтов на каждой боковой стороне
LAP_LID = 12.0         # нахлёст половин крышки (ступенька)
LAP_SHELF = 10.0       # нахлёст половин полки
LAP_GAP = 0.15         # зазор в нахлёсте
BOTTOM_RIBS = True     # рёбра на дне по эскизу (при печати нужны поддержки под дном)
OUT = pathlib.Path(__file__).parent / "out"

# =============================================================================
# ПРОИЗВОДНЫЕ
# =============================================================================

if BAT_ON_SIDE:
    PACK_Y, PACK_Z = BAT_THIN + BMS_T, BAT_WIDE   # BMS на широкой грани -> сбоку
else:
    PACK_Y, PACK_Z = BAT_WIDE, BAT_THIN + BMS_T

IN_W = max(W_MIN - 2 * WALL, PACK_Y + 2 * CLEAR, CTRL_W + 2 * CLEAR)
W = IN_W + 2 * WALL

FLOOR = WALL
CTRL_Z0 = FLOOR + MOUNT_H                         # низ контроллера
SHELF_Z = CTRL_Z0 + CTRL_H + CLEAR                # низ полки
BAT_Z0 = SHELF_Z + SHELF_T                        # дно отсека АКБ
BASE_H = BAT_Z0 + 1.0 + PACK_Z + CLEAR + LIP_H    # высота основания
H = BASE_H + LID_T

SLOPE = (TOP_L - BOT_L) / 2 / H                   # dx/dz наклонного торца
THETA = math.atan(SLOPE)
WALL_DX = WALL / math.cos(THETA)
# длина касания скругления низа на торце и на дне
R_BOTTOM_TAN = R_BOTTOM * math.tan((math.pi / 2 - THETA) / 2)


def outer_half_len(z: float) -> float:
    return BOT_L / 2 + SLOPE * z


def inner_half_len(z: float) -> float:
    return outer_half_len(z) - WALL_DX


CTRL_TOTAL_L = CTRL_L + 2 * CTRL_TAB_L
CTRL_X = -(CTRL_TOTAL_L / 2 + CTRL_CABLE_SPACE - (inner_half_len(CTRL_Z0) - CLEAR))
# батарея прижата к переднему торцу A; сзади (торец B) провода идут вверх к выводу
BAT_X = -(inner_half_len(BAT_Z0) - CLEAR - BAT_L / 2)

# =============================================================================
# ГЕОМЕТРИЯ: вспомогательное
# =============================================================================


def trapezoid(half_bot, half_top, z0, z1, width) -> cq.Workplane:
    pts = [(-half_bot, z0), (half_bot, z0), (half_top, z1), (-half_top, z1)]
    return cq.Workplane("XZ").polyline(pts).close().extrude(width / 2, both=True)


_OUTER = None


def outer_form() -> cq.Workplane:
    """Наружная форма основания со скруглениями (сплошная)."""
    global _OUTER
    if _OUTER is None:
        o = trapezoid(outer_half_len(0), outer_half_len(BASE_H), 0, BASE_H, W)
        o = o.edges("|Y and <Z").fillet(R_BOTTOM)
        side = [e for e in o.edges().vals()
                if abs(abs(e.Center().y) - W / 2) < 1e-3 and e.Center().z < BASE_H - 1e-3]
        _OUTER = o.newObject(side).fillet(R_SIDE)
    return _OUTER


def section(z: float) -> cq.Face:
    """Наружный контур корпуса на высоте z."""
    sl = outer_form().intersect(cq.Workplane().box(2000, 2000, 1).translate((0, 0, z + 0.5)))
    return sl.faces("<Z").val()


def offset_plate(z: float, offset: float, thickness: float, hole_offset: float | None = None) -> cq.Workplane:
    """Пластина по контуру сечения z, смещённому на offset (минус -- внутрь)."""
    w = section(z).outerWire().offset2D(offset, "arc")[0]
    inner = []
    if hole_offset is not None:
        inner = [section(z).outerWire().offset2D(hole_offset, "arc")[0]]
    f = cq.Face.makeFromWires(w, inner)
    s = cq.Solid.extrudeLinear(f, cq.Vector(0, 0, thickness))
    return cq.Workplane().add(s).translate((0, 0, -z))


def tapered_bar(length: float, profile: list[tuple[float, float]], taper: float) -> cq.Workplane:
    """Призма вдоль Z с профилем в XY и скошенными под 45° концами."""
    bar = cq.Workplane("XY").polyline(profile).close().extrude(length / 2, both=True)
    h = max(p[0] for p in profile)
    cutter = (cq.Workplane("XZ").polyline([(-0.01, -length / 2), (-0.01, length / 2),
                                           (h + 0.01, length / 2 - taper), (h + 0.01, -length / 2 + taper)])
              .close().extrude(50, both=True))
    return bar.intersect(cutter)


def rib_profile_out() -> list[tuple[float, float]]:
    """Трапециевидное ребро: x -- наружу от стенки, y -- поперёк."""
    return [(-0.3, -RIB_BASE / 2), (-0.3, RIB_BASE / 2), (RIB_H, RIB_TOP / 2), (RIB_H, -RIB_TOP / 2)]


def bottom_rib(y: float) -> cq.Workplane:
    length = BOT_L - 2 * R_BOTTOM_TAN - 16
    r = tapered_bar(length, rib_profile_out(), RIB_H)       # растёт по +X, длина по Z
    r = r.rotate((0, 0, 0), (0, 1, 0), 90)                  # длина -> X, «наружу» -> -Z
    return r.translate((0, y, 0))


def end_rib(sx: int, y: float) -> cq.Workplane:
    z0, z1 = R_BOTTOM_TAN + 6, BASE_H - 10
    length = (z1 - z0) / math.cos(THETA)
    r = tapered_bar(length, rib_profile_out(), RIB_H)
    if sx < 0:
        r = r.mirror("YZ")
    r = r.rotate((0, 0, 0), (0, 1, 0), sx * math.degrees(THETA))
    zc = (z0 + z1) / 2
    return r.translate((sx * outer_half_len(zc), y, zc))


def end_hole(sx: int, y: float, z: float, d: float) -> cq.Workplane:
    """Отверстие по нормали к наклонному торцу, с фаской снаружи."""
    c = cq.Workplane("YZ").circle(d / 2).extrude(15, both=True)
    cone = (cq.Workplane("YZ").circle(d / 2 + HOLE_CHAMFER).workplane(offset=HOLE_CHAMFER)
            .circle(d / 2).loft().translate((-HOLE_CHAMFER, 0, 0)))
    c = c.union(cone.translate((WALL_DX / 2 + 0.01, 0, 0)))
    if sx < 0:
        c = c.mirror("YZ")
    c = c.rotate((0, 0, 0), (0, 1, 0), sx * math.degrees(THETA))
    return c.translate((sx * (outer_half_len(z) - WALL_DX / 2), y, z))


# =============================================================================
# ЗАЩЁЛКИ («лягушка»)
# =============================================================================
# Детали строятся для стороны +Y и x=0 в координатах сборки, затем
# отражаются и разносятся по LATCH_X.

def _lat(y: float, z: float) -> tuple[float, float]:
    """Локальные (от стенки, от верха основания) -> абсолютные (y, z)."""
    return W / 2 + y, BASE_H + z


def latch_points() -> dict[str, tuple[float, float]]:
    """K -- центр перекладины серьги на зацепе, P1 -- ось рычага, P2 -- ось серьги."""
    k = (LATCH_KY, LID_T + 0.1 + BAIL_BAR_D / 2)
    p1 = LATCH_P1
    ux, uz = p1[0] - k[0], p1[1] - k[1]
    n = math.hypot(ux, uz)
    a = math.radians(LATCH_DELTA)  # повернуть к стенке (уменьшить y)
    vx = (ux * math.cos(a) - uz * math.sin(a)) / n
    vz = (ux * math.sin(a) + uz * math.cos(a)) / n
    if vx > ux / n:  # нужен поворот к стенке: y точки P2 меньше, чем на продолжении линии K-P1
        vx = (ux * math.cos(-a) - uz * math.sin(-a)) / n
        vz = (ux * math.sin(-a) + uz * math.cos(-a)) / n
    p2 = (p1[0] + LATCH_R * vx, p1[1] + LATCH_R * vz)
    return {"K": k, "P1": p1, "P2": p2}


def _x_extrude(sketch: cq.Workplane, x0: float, x1: float) -> cq.Workplane:
    return sketch.extrude(x1 - x0).translate((x0, 0, 0))


def _yz(points) -> cq.Workplane:
    return cq.Workplane("YZ").polyline([_lat(*p) for p in points]).close()


def _yz_circle(c, r) -> cq.Workplane:
    y, z = _lat(*c)
    return cq.Workplane("YZ").center(y, z).circle(r)


def latch_lug() -> cq.Workplane:
    """Проушина на стенке, снизу скос 45° -- печать без поддержек."""
    p = latch_points()
    y1, z1 = p["P1"]
    r = PIN_D / 2 + 2.8
    top = z1 + r
    tip = y1 + r
    prof = [(-0.5, top), (y1, top), (tip, z1), (tip, z1 - 1.0), (-0.5, z1 - 1.0 - (tip + 0.5))]
    lug = _x_extrude(_yz(prof), -LUG_HALF, LUG_HALF)
    lug = lug.union(_x_extrude(_yz_circle(p["P1"], r), -LUG_HALF, LUG_HALF))
    return lug.cut(_x_extrude(_yz_circle(p["P1"], PIN_D / 2 + 0.15), -20, 20))


def latch_lever() -> cq.Workplane:
    """Рычаг: две щеки по бокам проушины + ручка с упором для пальца."""
    p = latch_points()
    y1, z1 = p["P1"]
    x_in, x_out = LUG_HALF + GAP, LUG_HALF + GAP + CHEEK_T
    zb = z1 - 30.0
    cheek_prof = [(0.4, zb), (9.5, zb), (9.5, z1), (0.4, z1)]
    lever = None
    for sx in (-1, 1):
        c = _x_extrude(_yz(cheek_prof), x_in, x_out)
        c = c.union(_x_extrude(_yz_circle(p["P1"], PIN_D / 2 + 3.0), x_in, x_out))
        c = c.union(_x_extrude(_yz_circle(p["P2"], PEG_D / 2 + 2.2), x_in, x_out))
        if sx < 0:
            c = c.mirror("YZ")
        lever = c if lever is None else lever.union(c)
    web = _x_extrude(_yz([(5.5, zb), (9.5, zb), (9.5, z1 - 13), (5.5, z1 - 13)]), -x_out, x_out)
    lip = _x_extrude(_yz([(9.5, zb), (12.5, zb + 0.5), (12.5, zb + 3.5), (9.5, zb + 5.5)]), -x_out, x_out)
    lever = lever.union(web).union(lip)
    try:
        lever = lever.edges("|X").fillet(0.6)
    except Exception:
        pass
    lever = lever.cut(_x_extrude(_yz_circle(p["P1"], PIN_D / 2 + 0.15), -20, 20))  # ось -- плотная посадка
    lever = lever.cut(_x_extrude(_yz_circle(p["P2"], PEG_D / 2 + 0.15), -20, 20))  # под шипы серьги
    return lever


def latch_bail(preload: float = 0.0) -> cq.Workplane:
    """Серьга: два плеча снаружи щёк, шипы внутрь в рычаг, перекладина на зацеп."""
    p = latch_points()
    (y2, z2), (yk, zk) = p["P2"], p["K"]
    L = math.hypot(yk - y2, zk - z2) - preload
    ang = math.degrees(math.atan2(zk - z2, yk - y2))
    x_in = LUG_HALF + GAP + CHEEK_T + GAP
    x_out = x_in + BAIL_T
    # плечо в локальных координатах: вдоль оси (u) от P2 к K, ширина 5
    arm = (cq.Workplane("YZ").slot2D(L + BAIL_ARM_W, BAIL_ARM_W, 0).extrude(BAIL_T)
           .translate((0, L / 2, 0)))
    bar = cq.Workplane("YZ").circle(BAIL_BAR_D / 2).extrude(2 * x_out).translate((-x_out, L, 0))
    peg = cq.Workplane("YZ").circle(PEG_D / 2).extrude(CHEEK_T + GAP)
    bail = bar
    for sx in (-1, 1):
        a = arm.translate((x_in, 0, 0))
        pg = peg.translate((x_in - CHEEK_T - GAP, 0, 0))
        part = a.union(pg)
        if sx < 0:
            part = part.mirror("YZ")
        bail = bail.union(part)
    bail = bail.rotate((0, 0, 0), (1, 0, 0), ang)
    ya, za = _lat(y2, z2)
    return bail.translate((0, ya, za))


def latch_keeper() -> cq.Workplane:
    """Зацеп на кромке крышки (в координатах сборки); перекладина серьги ложится сверху."""
    half = LUG_HALF + GAP + CHEEK_T + GAP - 0.4
    ko = LATCH_KY + BAIL_BAR_D / 2 + 0.4  # упор сразу за перекладиной
    k = _x_extrude(_yz([(LID_OVER - 0.5, 0.0), (ko + 1.8, 0.0), (ko + 1.8, LID_T + 0.1),
                        (LID_OVER - 0.5, LID_T + 0.1)]), -half, half)
    lip = _x_extrude(_yz([(ko, LID_T + 0.1), (ko + 1.8, LID_T + 0.1), (ko + 1.8, LID_T + 2.8),
                          (ko + 0.3, LID_T + 2.8)]), -half, half)
    k = k.union(lip)
    # снизу 45°, чтобы не мешал при открывании и печатался
    cut = _x_extrude(_yz([(LID_OVER, -0.01), (ko + 1.9, -0.01), (ko + 1.9, 2.5)]), -20, 20)
    return k.cut(cut)


def _side(shape: cq.Workplane, x: float, sy: int) -> cq.Workplane:
    if sy < 0:
        shape = shape.mirror("XZ")
    return shape.translate((x, 0, 0))


def placed_latches(angle: float = 0.0) -> list[cq.Workplane]:
    """Рычаги и серьги всех защёлок в закрытом положении (angle -- поворот рычага)."""
    out = []
    p1 = _lat(*latch_points()["P1"])
    for x in LATCH_X:
        for sy in (-1, 1):
            lev = latch_lever()
            if angle:
                lev = lev.rotate((0, p1[0], p1[1]), (1, p1[0], p1[1]), angle)
            out.append(_side(lev, x, sy))
            if not angle:
                out.append(_side(latch_bail(), x, sy))
    return out


def latch_travel() -> list[tuple[float, float]]:
    """Растяжение серьги (мм) при открывании рычага на угол a (градусы)."""
    p = latch_points()
    (y1, z1), (y2, z2), (yk, zk) = p["P1"], p["P2"], p["K"]
    d0 = math.hypot(yk - y2, zk - z2)
    out = []
    for a in (0, 2, 4, 6, 8, 10, 12, 15, 20, 30):
        t = math.radians(a)  # поворот вокруг +X: низ рычага уходит от стенки
        dy, dz = y2 - y1, z2 - z1
        ny, nz = y1 + dy * math.cos(t) - dz * math.sin(t), z1 + dy * math.sin(t) + dz * math.cos(t)
        out.append((a, math.hypot(yk - ny, zk - nz) - d0))
    return out


# =============================================================================
# ДЕТАЛИ
# =============================================================================


def body() -> cq.Workplane:
    b = outer_form().faces(">Z").shell(-WALL)

    # уступы под полку (снизу 45° -- печать без поддержек), по прямым участкам стенок
    zl = SHELF_Z - LEDGE - 2
    l = 2 * (inner_half_len(SHELF_Z) - (R_SIDE - WALL)) - 4
    prof = [(0, 0), (LEDGE, LEDGE), (LEDGE, LEDGE + 2), (0, LEDGE + 2)]
    for sy in (-1, 1):
        led = cq.Workplane("YZ").polyline([(-0.5, 0)] + prof[1:] + [(-0.5, LEDGE + 2)]).close()
        led = led.extrude(l / 2, both=True)
        if sy > 0:
            led = led.mirror("XZ")
        b = b.union(led.translate((0, sy * IN_W / 2, zl)))

    # поперечные опоры под ушки контроллера с пилотными отверстиями (дно глухое)
    hx = CTRL_L / 2 + CTRL_TAB_L / 2
    for sx in (-1, 1):
        x = CTRL_X + sx * hx
        bar = (cq.Workplane("XY").box(CTRL_TAB_L + 4, IN_W - 2 * (R_SIDE - WALL) - 2, MOUNT_H + 0.5,
                                      centered=(True, True, False))
               .edges("|Y and >Z").chamfer(1.0).translate((x, 0, FLOOR - 0.5)))
        b = b.union(bar)
        for sy in (-1, 1):
            b = b.cut(cq.Workplane("XY").circle(PILOT_D / 2).extrude(PILOT_DEPTH)
                      .translate((x, sy * CTRL_HOLE_SPAN / 2, CTRL_Z0 - PILOT_DEPTH + 0.01)))

    # рёбра дна C и торцов A/B
    for sy in (-1, 1):
        if BOTTOM_RIBS:
            b = b.union(bottom_rib(sy * (W / 2 - RIB_EDGE_C - RIB_BASE / 2)))
        for sx in (-1, 1):
            b = b.union(end_rib(sx, sy * (RIB_GAP_AB / 2 + RIB_BASE / 2)))

    # петли с пазами под ремни (ремень идёт вертикально вдоль стенки)
    t = STRAP_T + LOOP_BAR
    hl = 20.0
    z_top = BASE_H - 12.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            prof = [(W / 2 - 0.5, z_top - hl - t), (W / 2 + t, z_top - hl), (W / 2 + t, z_top),
                    (W / 2 - 0.5, z_top)]
            loop = cq.Workplane("YZ").polyline(prof).close().extrude((STRAP_W + 2 * 5) / 2, both=True)
            try:
                loop = loop.edges("not |X").edges(">Y").fillet(1.5)
            except Exception:
                pass
            slot = (cq.Workplane("XY").rect(STRAP_W, STRAP_T).extrude(80)
                    .edges("|Z").fillet(1.2).translate((0, W / 2 + STRAP_T / 2, z_top - 60)))
            loop = loop.cut(slot)
            if sy < 0:
                loop = loop.mirror("XZ")
            b = b.union(loop.translate((sx * STRAP_X, 0, 0)))

    # фланец стыка двух половин (корпус режется по x=0 под стол принтера)
    b = b.union(joint_flange())
    for sy in (-1, 1):
        for z in JOINT_BOLT_Z:
            b = b.cut(cq.Workplane("YZ").circle(JOINT_BOLT_D / 2).extrude(20, both=True)
                      .translate((0, sy * (W / 2 + JOINT_FL_OUT / 2 + 0.5), z)))

    # проушины защёлок
    for x in LATCH_X:
        for sy in (-1, 1):
            b = b.union(_side(latch_lug(), x, sy))

    # выводы. Торец B (+X) -- задний, к сиденью: жгуты выходят ВВЕРХУ, у трубы,
    # и идут вдоль рамы. Зарядка ниже. Клапан на торце A.
    assert GLAND_M20 / 2 + HOLE_CHAMFER + 3 < RIB_GAP_AB / 2, "сальник M20 задевает рёбра торца"
    b = b.cut(end_hole(+1, 0, BASE_H - 20, GLAND_M20))
    b = b.cut(end_hole(+1, 0, BASE_H - 48, GLAND_M16))
    b = b.cut(end_hole(+1, 0, BAT_Z0 + 30, CHARGE_PORT))
    b = b.cut(end_hole(-1, 0, BAT_Z0 + PACK_Z * 0.6, VENT_M12))
    return b


def joint_flange() -> cq.Workplane:
    """U-образный наружный фланец по линии разреза x=0 (бока + низ)."""
    sl = outer_form().intersect(cq.Workplane().box(1, 2000, 2000).translate((0.5, 0, 0)))
    f = sl.faces("<X").val()
    w = f.outerWire().offset2D(JOINT_FL_OUT, "arc")[0]
    fl = cq.Workplane().add(cq.Solid.extrudeLinear(cq.Face.makeFromWires(w), cq.Vector(2 * JOINT_FL_T, 0, 0)))
    fl = fl.translate((-JOINT_FL_T, 0, 0)).cut(outer_form())
    z0 = -RIB_H if BOTTOM_RIBS else -0.01
    clip = cq.Workplane("XY").box(2 * JOINT_FL_T, W + 4 * JOINT_FL_OUT, BASE_H - 3 - z0,
                                  centered=(True, True, False)).translate((0, 0, z0))
    fl = fl.intersect(clip)
    try:
        fl = fl.edges("|X").fillet(2.0)
    except Exception:
        pass
    return fl


def _halfspace(x0: float, x1: float, z0: float = -1000, z1: float = 1000) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, 2000, z1 - z0, centered=False).translate((x0, -1000, z0))


def split_plain(shape: cq.Workplane) -> tuple[cq.Workplane, cq.Workplane]:
    """Разрез по x=0: (передняя половина, задняя половина)."""
    return shape.intersect(_halfspace(-1000, 0)), shape.intersect(_halfspace(0, 1000))


def split_lap(shape: cq.Workplane, lap: float, z_step: float) -> tuple[cq.Workplane, cq.Workplane]:
    """Разрез по x=0 со ступенчатым нахлёстом: передняя половина снизу, задняя сверху."""
    g, h = LAP_GAP, lap / 2
    front = shape.intersect(_halfspace(-1000, -h - g).union(_halfspace(-1000, h - g, -1000, z_step - g / 2)))
    rear = shape.intersect(_halfspace(h, 1000).union(_halfspace(-h, 1000, z_step + g / 2, 1000)))
    return front, rear


def lid() -> cq.Workplane:
    """Крышка в собственном нуле (z=0 -- нижняя плоскость пластины)."""
    l = offset_plate(BASE_H - 0.5, LID_OVER, LID_T)
    try:
        l = l.faces(">Z").edges().fillet(R_LID)
    except Exception:
        l = l.faces(">Z").edges().chamfer(R_LID)
    l = l.translate((0, 0, 0.5))  # offset_plate взят на 0.5 ниже верха -- контур тот же

    # вырезы свеса под ремни
    for sx in (-1, 1):
        for sy in (-1, 1):
            l = l.cut(cq.Workplane("XY").box(STRAP_W + 10, LID_OVER + 1, 20)
                      .translate((sx * STRAP_X, sy * (W / 2 + (LID_OVER + 1) / 2), 0)))

    # центрирующий буртик внутрь корпуса
    zb = BASE_H - LIP_H
    lip = offset_plate(zb, -(WALL_DX + 0.4), LIP_H, hole_offset=-(WALL_DX + 0.4 + LIP_T))
    l = l.union(lip.translate((0, 0, -LIP_H + 0.01)))

    # зацепы защёлок на кромке крышки
    for x in LATCH_X:
        for sy in (-1, 1):
            l = l.union(_side(latch_keeper(), x, sy).translate((0, 0, -BASE_H)))
    return l


def shelf() -> cq.Workplane:
    """Съёмная полка (z=0 -- низ): лежит на уступах, вырез под провода у торца B."""
    s = offset_plate(SHELF_Z, -(WALL_DX + 0.8), SHELF_T)
    s = s.faces(">Z").edges().fillet(0.8)
    xb = inner_half_len(SHELF_Z)
    s = s.cut(cq.Workplane("XY").rect(30, 40).extrude(10).edges("|Z").fillet(5)
              .translate((xb - 12, 0, -2)))
    for i in range(-4, 4):  # вентиляция и облегчение
        s = s.cut(cq.Workplane("XY").slot2D(IN_W - 34, 8, 90).extrude(10).translate((i * 26 - 10, 0, -2)))
    # пальцевый вырез, чтобы вынуть полку
    s = s.cut(cq.Workplane("XY").slot2D(30, 14, 0).extrude(10).translate((-xb + 30, 0, -2)))
    return s


# --- макеты содержимого -------------------------------------------------------

def dummy_controller() -> cq.Workplane:
    body_ = (cq.Workplane("XY").box(CTRL_L, CTRL_W, CTRL_H, centered=(True, True, False))
             .translate((CTRL_X, 0, CTRL_Z0)))
    tabs = (cq.Workplane("XY").box(CTRL_TOTAL_L, CTRL_W, 2.0, centered=(True, True, False))
            .translate((CTRL_X, 0, CTRL_Z0)))
    return body_.union(tabs)


def dummy_battery() -> tuple[cq.Workplane, cq.Workplane]:
    z = BAT_Z0 + 1.0
    if BAT_ON_SIDE:
        y_bat = IN_W / 2 - CLEAR - BAT_THIN / 2
        bat = (cq.Workplane("XY").box(BAT_L, BAT_THIN, BAT_WIDE, centered=(True, True, False))
               .translate((BAT_X, y_bat, z)))
        y_bms = y_bat - BAT_THIN / 2 - BMS_T / 2
        bms = (cq.Workplane("XY").box(BMS_L, BMS_T, BMS_W, centered=(True, True, False))
               .translate((BAT_X - (BAT_L - BMS_L) / 2 + 5, y_bms, z + (BAT_WIDE - BMS_W) / 2)))
    else:
        bat = (cq.Workplane("XY").box(BAT_L, BAT_WIDE, BAT_THIN, centered=(True, True, False))
               .translate((BAT_X, 0, z)))
        bms = (cq.Workplane("XY").box(BMS_L, BMS_W, BMS_T, centered=(True, True, False))
               .translate((BAT_X - (BAT_L - BMS_L) / 2 + 5, 0, z + BAT_THIN)))
    return bat, bms


# =============================================================================
# СБОРКА / ПРОВЕРКИ / ЭКСПОРТ
# =============================================================================

def build():
    y1, z1 = _lat(*latch_points()["P1"])
    parts = {"body": body(), "lid": lid(), "shelf": shelf(),
             # детали защёлки -- в нуле оси рычага
             "latch_lever": latch_lever().translate((0, -y1, -z1)),
             "latch_bail": latch_bail(LATCH_PRELOAD).translate((0, -y1, -z1))}
    parts["body_front"], parts["body_rear"] = split_plain(parts["body"])
    parts["lid_front"], parts["lid_rear"] = split_lap(parts["lid"], LAP_LID, LID_T / 2)
    parts["shelf_front"], parts["shelf_rear"] = split_lap(parts["shelf"], LAP_SHELF, SHELF_T / 2)
    bat, bms = dummy_battery()
    asm = cq.Assembly(name="ebike_frame_box")
    grey, dark, mid = cq.Color(0.55, 0.57, 0.6, 1), cq.Color(0.2, 0.2, 0.22, 1), cq.Color(0.4, 0.4, 0.45, 1)
    for half in ("front", "rear"):
        asm.add(parts[f"body_{half}"], name=f"body_{half}", color=grey)
        asm.add(parts[f"lid_{half}"], name=f"lid_{half}", loc=cq.Location((0, 0, BASE_H)), color=dark)
        asm.add(parts[f"shelf_{half}"], name=f"shelf_{half}", loc=cq.Location((0, 0, SHELF_Z)), color=mid)
    for i, l in enumerate(placed_latches()):
        asm.add(l, name=f"latch_part_{i + 1}", color=cq.Color(0.85, 0.55, 0.1, 1))
    asm.add(dummy_controller(), name="controller_dummy", color=cq.Color(0.75, 0.3, 0.1, 1))
    asm.add(bat, name="battery_dummy", color=cq.Color(0.1, 0.45, 0.8, 1))
    asm.add(bms, name="bms_dummy", color=cq.Color(0.1, 0.6, 0.2, 1))
    return parts, asm


def _vol(a: cq.Workplane, b: cq.Workplane) -> float:
    try:
        return sum(s.Volume() for s in a.intersect(b).solids().vals())
    except Exception:
        return 0.0


def checks(parts) -> list[str]:
    bat, bms = dummy_battery()
    ctrl = dummy_controller()
    lid_p = parts["lid"].translate((0, 0, BASE_H))
    shelf_p = parts["shelf"].translate((0, 0, SHELF_Z))
    pairs = {
        "контроллер/корпус": (ctrl, parts["body"]), "контроллер/полка": (ctrl, shelf_p),
        "АКБ/корпус": (bat, parts["body"]), "АКБ/полка": (bat, shelf_p), "АКБ/крышка": (bat, lid_p),
        "BMS/корпус": (bms, parts["body"]), "BMS/полка": (bms, shelf_p), "BMS/крышка": (bms, lid_p),
        "полка/корпус": (shelf_p, parts["body"]), "крышка/корпус": (lid_p, parts["body"]),
    }
    for i, l in enumerate(placed_latches()):
        kind = "рычаг" if i % 2 == 0 else "серьга"
        n = i // 2 + 1
        pairs[f"защёлка {n} {kind}/корпус"] = (l, parts["body"])
        pairs[f"защёлка {n} {kind}/крышка"] = (l, lid_p)
    out = []
    for n, (a, b) in pairs.items():
        v = _vol(a, b)
        out.append(f"  {n:26s} {'OK' if v < 0.5 else f'ПЕРЕСЕЧЕНИЕ {v:.0f} мм3'}")
    lev, bail = placed_latches()[0], placed_latches()[1]
    v = _vol(lev, bail)
    out.append(f"  {'рычаг/серьга':26s} {'OK' if v < 0.5 else f'ПЕРЕСЕЧЕНИЕ {v:.0f} мм3'}")
    for a in (5, 15, 30, 60):
        v = _vol(placed_latches(a)[0], parts["body"])
        out.append(f"  {'рычаг открыт на %d°/корпус' % a:26s} {'OK' if v < 0.5 else f'ПЕРЕСЕЧЕНИЕ {v:.0f} мм3'}")
    tr = latch_travel()
    peak = max(tr, key=lambda t: t[1])
    out.append("  мёртвая точка: серьга растягивается на %.2f мм при %d° -- %s"
               % (peak[1], peak[0], "защёлка держит сама" if peak[1] > 0.1 else "НЕТ ФИКСАЦИИ"))
    out.append("  натяг серьги в закрытом положении: %.1f мм (печатная серьга короче)" % LATCH_PRELOAD)

    room_b = inner_half_len(CTRL_Z0) - (CTRL_X + CTRL_TOTAL_L / 2)
    out.append(f"  место под провода контроллера у торца B: {room_b:.0f} мм")
    room_b2 = inner_half_len(BAT_Z0) - (BAT_X + BAT_L / 2)
    out.append(f"  место под провода за батареей у торца B: {room_b2:.0f} мм внизу отсека, "
               f"{inner_half_len(BASE_H - 20) - (BAT_X + BAT_L / 2):.0f} мм у вывода")
    return out


PRINT_PARTS = {  # что печатать и сколько штук; ориентация -- см. README
    "body_front": 1, "body_rear": 1, "lid_front": 1, "lid_rear": 1,
    "shelf_front": 1, "shelf_rear": 1, "latch_lever": 4, "latch_bail": 4,
}


def report(parts) -> str:
    lines = [
        f"Корпус: верх {TOP_L:.0f}, низ {BOT_L:.0f}, высота {H:.0f}, ширина {W:.0f} мм "
        f"(+ рёбра {RIB_H:.0f} мм; петли, замки и фланец стыка выступают на ~{JOINT_FL_OUT:.0f} мм)",
        f"Батарея {'на боку' if BAT_ON_SIDE else 'плашмя'}: с BMS занимает "
        f"{PACK_Y:.0f} мм поперёк и {PACK_Z:.0f} мм по высоте",
        f"Полка на высоте {SHELF_Z:.0f} мм",
        "",
        f"Детали для печати (стол {PRINT_BED[0]:.0f} x {PRINT_BED[1]:.0f} x {PRINT_BED[2]:.0f}), "
        "габарит X x Y x Z, мм:",
    ]
    for name, qty in PRINT_PARTS.items():
        bb = parts[name].val().BoundingBox()
        a, b = max(bb.xlen, bb.ylen), min(bb.xlen, bb.ylen)
        fits = a <= max(PRINT_BED[:2]) and b <= min(PRINT_BED[:2]) and bb.zlen <= PRINT_BED[2]
        lines.append(f"  {name:12s} x{qty}  {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:6.1f}  "
                     f"{'влезает' if fits else '!! НЕ ВЛЕЗАЕТ'}")
    lines += ["", "Проверки:"] + checks(parts)
    # половины не пересекаются и вместе дают целую деталь
    for n in ("body", "lid", "shelf"):
        f, r = parts[f"{n}_front"], parts[f"{n}_rear"]
        ov = _vol(f, r)
        lost = parts[n].val().Volume() - f.val().Volume() - r.val().Volume()
        lines.append(f"  {n + ': половины':26s} {'OK' if ov < 0.5 else f'ПЕРЕСЕЧЕНИЕ {ov:.0f} мм3'}"
                     f" (зазоры нахлёста {lost:.0f} мм3)")
    return "\n".join(lines)


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob("*.st[lp]"):
        old.unlink()
    parts, asm = build()
    for name in PRINT_PARTS:
        cq.exporters.export(parts[name], str(OUT / f"{name}.stl"), tolerance=0.03, angularTolerance=0.1)
        cq.exporters.export(parts[name], str(OUT / f"{name}.step"))
    asm.export(str(OUT / "assembly.step"))
    txt = report(parts)
    (OUT / "dimensions.txt").write_text(txt + "\n", encoding="utf-8")
    print(txt)
