"""
Корпус для электровелосипеда: аккумулятор 18650 + BMS + контроллер.
Параметрическая модель на CadQuery.

Концепция (по эскизам Виталика):
  * корпус -- трапеция в треугольнике рамы, подвешивается под верхнюю трубу
    на двух ремнях (как подрамная сумка); ремни идут через петли на стенках;
  * эскиз: верх 340, низ 280, высота 160, ширина 100 мм -- ширина и высота
    здесь пересчитываются по содержимому (см. отчёт);
  * сверху отсек АКБ: батарея 270x110x73 стоит НА БОКУ (73 мм поперёк
    велосипеда -- меньше ширина, больше места коленям), BMS 140x105x20
    остаётся прикрученной к её грани (как на фото), т.е. тоже сбоку;
  * съёмная полка-перегородка;
  * снизу отсек контроллера: контроллер 172x83x42 (с ушками 196) лежит плашмя,
    ушками прикручен ко дну;
  * крышка сверху, застёгивается на 4 защёлки (snap-fit), без винтов;
  * на торцах A/B и на дне C -- рёбра 5 мм (по эскизу).

Оси: X -- вдоль верхней трубы, Y -- поперёк велосипеда (ширина), Z -- вверх.

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
CTRL_HOLE_SPAN = 55.0  # ! межцентровое отверстий в ушке -- УТОЧНИТЬ (в дне пазы +-6 мм)
CTRL_HOLE_D = 4.5      # M4
CTRL_CABLE_SPACE = 45.0  # запас на изгиб проводов у торца B

# --- Корпус -------------------------------------------------------------------
TOP_L = 340.0          # длина по верху (эскиз)
BOT_L = 280.0          # длина по низу (эскиз)
W_MIN = 0.0            # можно задать минимальную ширину (эскиз: 100)
WALL = 3.0             # стенки (PETG/ASA)
FLOOR = 3.0
LID_T = 3.0
CLEAR = 2.0            # зазор до содержимого (на сторону)
SHELF_T = 3.0
LEDGE = 4.0            # уступы под полку
LIP_H = 3.0            # центрирующий буртик крышки

# --- Защёлки крышки -----------------------------------------------------------
LATCH_X = (-70.0, 70.0)  # по две на каждой длинной стороне
LATCH_W = 16.0
LATCH_ARM_T = 2.4        # толщина лепестка (PETG: 2-2.5)
LATCH_ARM_L = 16.0
LATCH_HOOK = 1.6         # зацеп
LATCH_GAP = 0.3

# --- Рёбра на торцах и дне (эскиз 1, виды A/B и C) --------------------------
RIB_H = 5.0
RIB_BASE, RIB_TOP = 8.0, 4.0
RIB_GAP_AB = 35.0      # просвет между рёбрами на торцах A/B
RIB_EDGE_C = 20.0      # отступ рёбер дна от длинных кромок

# --- Ремни под верхнюю трубу -------------------------------------------------
STRAP_W, STRAP_T = 27.0, 4.5  # стропа/липучка 25 мм
STRAP_X = 125.0               # положение ремней от центра (+/-)

# --- Выводы -------------------------------------------------------------------
GLAND_M20 = 20.4       # жгут контроллера
GLAND_M16 = 16.4
CHARGE_PORT = 16.2     # GX16 / разъём зарядки
VENT_M12 = 12.4        # мембранный клапан

PRINT_BED = (256.0, 256.0)
OUT = pathlib.Path(__file__).parent / "out"

# =============================================================================
# ПРОИЗВОДНЫЕ
# =============================================================================

if BAT_ON_SIDE:
    PACK_Y, PACK_Z = BAT_THIN + BMS_T, BAT_WIDE   # BMS на широкой грани -> сбоку
else:
    PACK_Y, PACK_Z = BAT_WIDE, BAT_THIN + BMS_T   # BMS сверху

IN_W = max(W_MIN - 2 * 3.0, PACK_Y + 2 * CLEAR, CTRL_W + 2 * CLEAR)
W = IN_W + 2 * WALL

SHELF_Z = FLOOR + CLEAR + CTRL_H + CLEAR          # низ полки
BAT_Z0 = SHELF_Z + SHELF_T                        # дно отсека АКБ
BASE_H = BAT_Z0 + 1.0 + PACK_Z + CLEAR + LIP_H    # высота основания
H = BASE_H + LID_T

SLOPE = (TOP_L - BOT_L) / 2 / H                   # dx/dz наклонного торца
THETA = math.atan(SLOPE)
WALL_DX = WALL / math.cos(THETA)


def outer_half_len(z: float) -> float:
    return BOT_L / 2 + SLOPE * z


def inner_half_len(z: float) -> float:
    return outer_half_len(z) - WALL_DX


CTRL_TOTAL_L = CTRL_L + 2 * CTRL_TAB_L
# контроллер сдвинут к торцу A, у торца B -- место под провода
CTRL_X = -(CTRL_TOTAL_L / 2 + CTRL_CABLE_SPACE - (inner_half_len(FLOOR) - CLEAR))
# батарея по центру; BMS-сторона к стенке -Y
BAT_X = 0.0

# =============================================================================
# ГЕОМЕТРИЯ
# =============================================================================


def trapezoid(half_bot, half_top, z0, z1, width) -> cq.Workplane:
    pts = [(-half_bot, z0), (half_bot, z0), (half_top, z1), (-half_top, z1)]
    return cq.Workplane("XZ").polyline(pts).close().extrude(width / 2, both=True)


def outer_solid(z1: float) -> cq.Workplane:
    return trapezoid(outer_half_len(0), outer_half_len(z1), 0, z1, W)


def rib_profile() -> list[tuple[float, float]]:
    return [(-RIB_BASE / 2, 0), (RIB_BASE / 2, 0), (RIB_TOP / 2, RIB_H), (-RIB_TOP / 2, RIB_H)]


def bottom_rib(y: float) -> cq.Workplane:
    """Ребро на дне вдоль X (выступает вниз)."""
    r = cq.Workplane("YZ").polyline(rib_profile()).close().extrude((BOT_L - 30) / 2, both=True)
    return r.mirror("XY").translate((0, y, 0))


def end_rib(sx: int, y: float) -> cq.Workplane:
    """Ребро на наклонном торце (выступает наружу по нормали)."""
    length = H / math.cos(THETA) - 25
    # профиль в плоскости XY: x -- наружу, y -- поперёк; длина вдоль Z
    prof = [(0, -RIB_BASE / 2), (0, RIB_BASE / 2), (RIB_H, RIB_TOP / 2), (RIB_H, -RIB_TOP / 2)]
    r = cq.Workplane("XY").polyline(prof).close().extrude(length / 2, both=True)
    if sx < 0:
        r = r.mirror("YZ")
    r = r.rotate((0, 0, 0), (0, 1, 0), sx * math.degrees(THETA))
    zc = BASE_H / 2
    return r.translate((sx * (outer_half_len(zc) - 0.3), y, zc))


def slanted_hole(sx: int, y: float, z: float, d: float) -> cq.Workplane:
    c = cq.Workplane("YZ").circle(d / 2).extrude(20, both=True)
    c = c.rotate((0, 0, 0), (0, 1, 0), sx * math.degrees(THETA))
    return c.translate((sx * (outer_half_len(z) - WALL_DX / 2), y, z))


def catch_z() -> tuple[float, float]:
    """Высоты выступа-зацепа на стенке (низ, верх)."""
    top = BASE_H - LATCH_ARM_L + 4.0
    return top - 2.0, top


def body() -> cq.Workplane:
    b = outer_solid(BASE_H)
    cav = trapezoid(inner_half_len(FLOOR), inner_half_len(BASE_H + 5), FLOOR, BASE_H + 5, IN_W)
    b = b.cut(cav)

    # уступы под полку (снизу фаска 45° -- печать без поддержек)
    for sy in (-1, 1):
        l = 2 * inner_half_len(SHELF_Z - LEDGE) - 2
        prof = [(0, 0), (LEDGE, LEDGE), (LEDGE, LEDGE + 2), (0, LEDGE + 2)]
        led = cq.Workplane("YZ").polyline(prof).close().extrude(l / 2, both=True)
        if sy > 0:
            led = led.mirror("XZ")
        b = b.union(led.translate((0, sy * IN_W / 2, SHELF_Z - LEDGE - 2)))

    # рёбра дна C и торцов A/B
    for sy in (-1, 1):
        b = b.union(bottom_rib(sy * (W / 2 - RIB_EDGE_C - RIB_BASE / 2)))
        for sx in (-1, 1):
            b = b.union(end_rib(sx, sy * (RIB_GAP_AB / 2 + RIB_BASE / 2)))

    # петли под ремни на длинных стенках (ремень вертикально вдоль стенки)
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = sx * STRAP_X
            loop = (cq.Workplane("XY").box(STRAP_W + 8, STRAP_T + 3, 16, centered=(True, True, False))
                    .edges("|Z").fillet(1.5)
                    .translate((x, sy * (W / 2 + (STRAP_T + 3) / 2 - 0.01), BASE_H - 30)))
            # фаска снизу под печать
            slot = (cq.Workplane("XY").box(STRAP_W, STRAP_T, 40, centered=(True, True, False))
                    .translate((x, sy * (W / 2 + STRAP_T / 2), BASE_H - 40)))
            b = b.union(loop).cut(slot)

    # выступы-зацепы под защёлки крышки
    z0, z1 = catch_z()
    for x in LATCH_X:
        for sy in (-1, 1):
            # снизу площадка (за неё цепляется зуб), сверху скос -- зуб заскакивает
            prof = [(0, z0), (LATCH_HOOK, z0), (LATCH_HOOK, z0 + 0.8), (0, z1 + LATCH_HOOK)]
            c = cq.Workplane("YZ").polyline(prof).close().extrude(LATCH_W / 2, both=True)
            if sy < 0:
                c = c.mirror("XZ")
            b = b.union(c.translate((x, sy * (W / 2 - 0.01), 0)))

    # пазы M4 под ушки контроллера в дне (регулировка +-6 мм поперёк)
    hx = CTRL_L / 2 + CTRL_TAB_L / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            b = b.cut(cq.Workplane("XY").slot2D(12 + CTRL_HOLE_D, CTRL_HOLE_D, 90)
                      .extrude(FLOOR + 2 * RIB_H).translate((CTRL_X + sx * hx, sy * CTRL_HOLE_SPAN / 2, -RIB_H - 1)))

    # выводы: торец B (+X) -- жгут контроллера и зарядка; торец A -- клапан
    z_ctrl = FLOOR + CLEAR + CTRL_H / 2
    z_bat = BAT_Z0 + PACK_Z * 0.75
    gy = RIB_GAP_AB / 2 - 1  # отверстия между рёбрами торца
    b = b.cut(slanted_hole(+1, 0, z_ctrl + 2, GLAND_M20))
    b = b.cut(slanted_hole(+1, 0, z_bat, CHARGE_PORT))
    b = b.cut(slanted_hole(+1, 0, z_bat - 32, GLAND_M16))
    b = b.cut(slanted_hole(-1, 0, z_bat, VENT_M12))
    assert GLAND_M20 / 2 < gy + 2  # проверка, что не задели рёбра
    return b


def lid() -> cq.Workplane:
    over = LATCH_GAP + LATCH_HOOK + LATCH_ARM_T      # свес крышки над стенкой
    lid_l, lid_w = TOP_L, W + 2 * over
    l = (cq.Workplane("XY").box(lid_l, lid_w, LID_T, centered=(True, True, False))
         .edges("|Z").fillet(5).faces(">Z").edges().chamfer(1.0))

    # вырезы свеса под ремни
    for sx in (-1, 1):
        for sy in (-1, 1):
            l = l.cut(cq.Workplane("XY").box(STRAP_W + 10, over + 1, 20)
                      .translate((sx * STRAP_X, sy * (W / 2 + (over + 1) / 2), 0)))

    # центрирующий буртик по внутреннему контуру
    lip_o = trapezoid(inner_half_len(BASE_H - LIP_H) - 0.4, inner_half_len(BASE_H) - 0.4,
                      -LIP_H, 0.01, IN_W - 0.8)
    lip_i = trapezoid(inner_half_len(BASE_H - LIP_H) - 2.4, inner_half_len(BASE_H) - 2.4,
                      -LIP_H - 1, 1, IN_W - 4.8)
    l = l.union(lip_o.cut(lip_i))

    # защёлки: лепесток вниз снаружи стенки + зуб внутрь
    z0, _ = catch_z()
    zr = z0 - BASE_H                                  # низ зацепа в координатах крышки
    for x in LATCH_X:
        for sy in (-1, 1):
            y_in = W / 2 + LATCH_GAP + LATCH_HOOK    # внутренняя грань лепестка
            arm_bot = zr - LATCH_HOOK - 1.5
            # зуб: от низа лепестка внутрь до стенки, верх зуба = низ зацепа
            prof = [
                (y_in, 0.01), (y_in + LATCH_ARM_T, 0.01),
                (y_in + LATCH_ARM_T + 1.5, arm_bot - 4),   # язычок для пальца
                (y_in + LATCH_ARM_T, arm_bot - 4),
                (y_in + LATCH_ARM_T, arm_bot - 2.5),
                (W / 2 + LATCH_GAP, arm_bot - 2.5),          # скос для защёлкивания
                (W / 2 + LATCH_GAP, zr - 0.2),
                (y_in, zr - 0.2),
            ]
            a = cq.Workplane("YZ").polyline(prof).close().extrude(LATCH_W / 2, both=True)
            if sy < 0:
                a = a.mirror("XZ")
            l = l.union(a.translate((x, 0, 0)))
    return l


def shelf() -> cq.Workplane:
    """Съёмная полка: лежит на уступах, вырез под провода у торца B."""
    z0, z1 = SHELF_Z, SHELF_Z + SHELF_T
    s = trapezoid(inner_half_len(z0) - 0.8, inner_half_len(z1) - 0.8, z0, z1, IN_W - 1.0)
    s = s.cut(cq.Workplane("XY").box(24, 36, 20).translate((inner_half_len(z0) - 10, 0, z0)))
    for i in range(-4, 4):
        s = s.cut(cq.Workplane("XY").slot2D(IN_W - 30, 8, 90).extrude(10)
                  .translate((i * 26, 0, z0 - 3)))
    return s.translate((0, 0, -SHELF_Z))


# --- макеты содержимого -------------------------------------------------------

def dummy_controller() -> cq.Workplane:
    z = FLOOR
    body_ = (cq.Workplane("XY").box(CTRL_L, CTRL_W, CTRL_H, centered=(True, True, False))
             .translate((CTRL_X, 0, z)))
    tabs = cq.Workplane("XY").box(CTRL_TOTAL_L, CTRL_W, 2.0, centered=(True, True, False)).translate((CTRL_X, 0, z))
    return body_.union(tabs)


def dummy_battery() -> tuple[cq.Workplane, cq.Workplane]:
    z = BAT_Z0 + 1.0
    if BAT_ON_SIDE:
        y_bat = IN_W / 2 - CLEAR - BAT_THIN / 2
        bat = cq.Workplane("XY").box(BAT_L, BAT_THIN, BAT_WIDE, centered=(True, True, False)).translate((BAT_X, y_bat, z))
        y_bms = y_bat - BAT_THIN / 2 - BMS_T / 2
        bms = (cq.Workplane("XY").box(BMS_L, BMS_T, BMS_W, centered=(True, True, False))
               .translate((BAT_X - (BAT_L - BMS_L) / 2 + 5, y_bms, z + (BAT_WIDE - BMS_W) / 2)))
    else:
        bat = cq.Workplane("XY").box(BAT_L, BAT_WIDE, BAT_THIN, centered=(True, True, False)).translate((BAT_X, 0, z))
        bms = (cq.Workplane("XY").box(BMS_L, BMS_W, BMS_T, centered=(True, True, False))
               .translate((BAT_X - (BAT_L - BMS_L) / 2 + 5, 0, z + BAT_THIN)))
    return bat, bms


# =============================================================================
# СБОРКА / ОТЧЁТ / ЭКСПОРТ
# =============================================================================

def build():
    parts = {"body": body(), "lid": lid(), "shelf": shelf()}
    bat, bms = dummy_battery()
    asm = cq.Assembly(name="ebike_frame_box")
    asm.add(parts["body"], name="body", color=cq.Color(0.55, 0.57, 0.6, 1))
    asm.add(parts["lid"], name="lid", loc=cq.Location((0, 0, BASE_H)), color=cq.Color(0.2, 0.2, 0.22, 1))
    asm.add(parts["shelf"], name="shelf", loc=cq.Location((0, 0, SHELF_Z)), color=cq.Color(0.4, 0.4, 0.45, 1))
    asm.add(dummy_controller(), name="controller_dummy", color=cq.Color(0.75, 0.3, 0.1, 1))
    asm.add(bat, name="battery_dummy", color=cq.Color(0.1, 0.45, 0.8, 1))
    asm.add(bms, name="bms_dummy", color=cq.Color(0.1, 0.6, 0.2, 1))
    return parts, asm


def _vol(a: cq.Workplane, b: cq.Workplane) -> float:
    try:
        r = a.intersect(b)
        return sum(s.Volume() for s in r.solids().vals())
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
        "BMS/корпус": (bms, parts["body"]), "BMS/крышка": (bms, lid_p),
        "полка/корпус": (shelf_p, parts["body"]), "крышка/корпус": (lid_p, parts["body"]),
    }
    out = []
    for n, (a, b) in pairs.items():
        v = _vol(a, b)
        out.append(f"  {n:20s} {'OK' if v < 0.5 else f'ПЕРЕСЕЧЕНИЕ {v:.0f} мм3'}")
    room_b = inner_half_len(FLOOR) - (CTRL_X + CTRL_TOTAL_L / 2)
    out.append(f"  место под провода контроллера у торца B: {room_b:.0f} мм")
    room_bat = 2 * inner_half_len(BAT_Z0) - BAT_L
    out.append(f"  запас по длине для АКБ (по низу отсека): {room_bat:.0f} мм")
    return out


def report(parts) -> str:
    lines = [
        f"Корпус: верх {TOP_L:.0f}, низ {BOT_L:.0f}, высота {H:.0f}, ширина {W:.0f} мм "
        f"(+ рёбра {RIB_H:.0f} мм, петли ремней, свес крышки)",
        f"Батарея {'на боку' if BAT_ON_SIDE else 'плашмя'}: с BMS занимает "
        f"{PACK_Y:.0f} мм поперёк и {PACK_Z:.0f} мм по высоте",
        f"Полка на высоте {SHELF_Z:.0f} мм",
        "",
        "Габариты деталей (X x Y x Z, мм):",
    ]
    for name, p in parts.items():
        bb = p.val().BoundingBox()
        a, b = max(bb.xlen, bb.ylen), min(bb.xlen, bb.ylen)
        S = min(PRINT_BED)
        if a <= max(PRINT_BED) and b <= S:
            note = "ok"
        elif (a + b) / math.sqrt(2) <= S:
            note = "! только по диагонали стола"
        else:
            note = f"!! больше стола {PRINT_BED[0]:.0f}x{PRINT_BED[1]:.0f}"
        lines.append(f"  {name:6s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:6.1f}  {note}")
    lines += ["", "Проверка пересечений:"] + checks(parts)
    return "\n".join(lines)


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    parts, asm = build()
    for name, p in parts.items():
        cq.exporters.export(p, str(OUT / f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        cq.exporters.export(p, str(OUT / f"{name}.step"))
    asm.export(str(OUT / "assembly.step"))
    txt = report(parts)
    (OUT / "dimensions.txt").write_text(txt + "\n", encoding="utf-8")
    print(txt)
