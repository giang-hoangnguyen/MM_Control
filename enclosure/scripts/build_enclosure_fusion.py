"""
Dựng vỏ 2 mảnh (Enclosure_Top, Enclosure_Bottom) cho board MM_Control trong Autodesk Fusion.

Cách dùng: mở file MM_Control_Board_v7.f3d (có mô hình board dạng mesh), chạy script này
trong Fusion (Utilities > Scripts and Add-Ins) hoặc qua Fusion MCP. Script xóa component
'3D_Printing_Components' cũ rồi dựng lại, nên chạy lại được.

Script đọc bounding box của các mesh LED (MCU1, 5V1, 3V3, STEP_EN1, DC_Fault1-4, VCC1)
trong file để cắt rãnh LED. Chạy trong file khác thì cần các mesh cùng tên, hoặc bỏ phần 'groups'.

Đơn vị trong script: mm (đổi sang cm khi gọi API Fusion). Trục Z hướng lên, PCB nằm z = 0 .. 1,6 mm,
tâm board ở gốc tọa độ, board 100 x 100 mm, lỗ vít Ø4,2 mm tại (±45, ±45).
"""
import adsk.core, adsk.fusion, json
A = adsk.core
F = adsk.fusion


def run(context):
    app = A.Application.get()
    des = F.Design.cast(app.activeProduct)
    root = des.rootComponent
    bm = F.TemporaryBRepManager.get()
    BT = F.BooleanTypes

    # xóa bản dựng cũ
    for i in range(root.occurrences.count - 1, -1, -1):
        o = root.occurrences.item(i)
        if o.component.name == '3D_Printing_Components':
            o.deleteMe()

    def P(x, y, z):
        return A.Point3D.create(x / 10.0, y / 10.0, z / 10.0)

    def box(x0, x1, y0, y1, z0, z1):
        o = A.OrientedBoundingBox3D.create(
            P((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
            A.Vector3D.create(1, 0, 0), A.Vector3D.create(0, 1, 0),
            (x1 - x0) / 10.0, (y1 - y0) / 10.0, (z1 - z0) / 10.0)
        return bm.createBox(o)

    def cyl(x, y, z0, z1, r):
        return bm.createCylinderOrCone(P(x, y, z0), r / 10.0, P(x, y, z1), r / 10.0)

    def U(a, b):
        assert bm.booleanOperation(a, b, BT.UnionBooleanType)
        return a

    def D(a, b):
        assert bm.booleanOperation(a, b, BT.DifferenceBooleanType)
        return a

    def rrect(h, R, z0, z1):
        """Hình vuông cạnh 2h, bo góc R (tâm tại gốc)."""
        a = box(-h, h, -(h - R), (h - R), z0, z1)
        U(a, box(-(h - R), (h - R), -h, h, z0, z1))
        for sx in (-1, 1):
            for sy in (-1, 1):
                U(a, cyl(sx * (h - R), sy * (h - R), z0, z1, R))
        return a

    def rbox(x0, x1, y0, y1, R, z0, z1):
        """Hộp chữ nhật bo 4 góc đứng bán kính R (dùng cho khoang tay thao tác)."""
        a = box(x0 + R, x1 - R, y0, y1, z0, z1)
        U(a, box(x0, x1, y0 + R, y1 - R, z0, z1))
        for cx in (x0 + R, x1 - R):
            for cy in (y0 + R, y1 - R):
                U(a, cyl(cx, cy, z0, z1, R))
        return a

    # ---- thông số (mm) ----
    CH, RC = 50.5, 4.5            # khoang trong: nửa cạnh, bán kính góc
    T = 2.4                       # dày vách
    OH, RO = CH + T, RC + T       # ngoài: nửa cạnh 52,9; bán kính góc 6,9
    ZF, ZB = -5.5, -8.5           # mặt trong sàn đế, mặt dưới đế
    ZR, ZT = 19.8, 22.2           # mặt trong mái nắp, mặt ngoài mái nắp
    POST_R, SCREW_HOLE, CB_R, CB_D = 5.5, 2.25, 4.0, 4.5   # trụ, lỗ vít M4 Ø4,5, lỗ lõm Ø8 sâu 4,5
    INS_R, INS_DEPTH = 2.8, 9.0   # lỗ heat-set insert M4: Ø5,6, sâu 9
    HC = 45.0                     # vị trí lỗ vít (±45)
    FILL = 50.8
    BR = 2.0                      # bán kính bo góc khoang tay thao tác

    # ---- đế ----
    base = rrect(OH, RO, ZB, 0.0)
    D(base, rrect(CH, RC, ZF, 1.0))
    ring = D(rrect(51.5, RC + 1.0, -0.5, 3.0), rrect(CH, RC, -1.0, 3.5))   # gờ định vị (tongue)
    U(base, ring)
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * HC, sy * HC
            U(base, cyl(cx, cy, ZF - 0.5, 0.0, POST_R))                    # standoff
            U(base, box(min(cx, sx * FILL), max(cx, sx * FILL), min(cy, sy * FILL), max(cy, sy * FILL), ZF - 0.5, 0.0))
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * HC, sy * HC
            D(base, cyl(cx, cy, ZB - 1.0, 1.0, SCREW_HOLE))
            D(base, cyl(cx, cy, ZB - 1.0, ZB + CB_D, CB_R))

    # ---- nắp ----
    lid = rrect(OH, RO, 0.0, ZT)
    D(lid, rrect(CH, RC, -1.0, ZR))
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * HC, sy * HC
            U(lid, cyl(cx, cy, 1.6, ZR + 0.5, POST_R))
            U(lid, box(min(cx, sx * FILL), max(cx, sx * FILL), min(cy, sy * FILL), max(cy, sy * FILL), 1.6, ZR + 0.5))
    for sx in (-1, 1):
        for sy in (-1, 1):
            D(lid, cyl(sx * HC, sy * HC, 1.0, 1.6 + INS_DEPTH, INS_R))
    D(lid, D(rrect(51.8, RC + 1.3, -1.0, 3.3), rrect(CH, RC, -2.0, 4.3)))   # rãnh khớp với gờ của đế

    # ---- cutout ----
    RZ0, RZ1 = ZR - 1.0, ZT + 1.0
    WZ1 = 11.5
    big = []     # khoang tay thao tác trên mái: bo góc R2
    small = []   # khe thông gió, rãnh LED: không bo
    wall = []    # cửa sổ ở vách (cắt cả nắp và đế)
    for (y0, y1) in ((12.32, 23.22), (27.58, 38.48), (-23.72, -12.82)):      # 3 terminal bên +X
        wall.append((48.0, 54.0, y0 - 0.5, y1 + 0.5))
        big.append((39.5, 48.5, y0 - 0.3, y1 + 0.3))
    for (y0, y1) in ((7.85, 18.75), (23.09, 33.98)):                          # 2 terminal bên -X
        wall.append((-54.0, -48.0, y0 - 0.5, y1 + 0.5))
    wall.append((-18.2, -6.7, 48.0, 54.0))                                    # CAN (J17)
    big.append((-17.7, -7.2, 41.5, 48.5))
    big.append((-50.5, -28.0, -24.0, 34.5))                                   # khoang cổng XH (stepper) + servo
    big.append((-37.5, 37.5, -50.5, -37.0))                                   # khoang phía -Y
    big.append((-37.5, -20.0, 38.0, 50.5))                                    # SWD
    big.append((16.5, 37.0, 38.0, 50.5))                                      # BOOT0 / RESET
    for xc in (25.6, 3.88, -17.25):                                           # khe thông gió trên 3 TMC2209
        for k in (-8, -4, 0, 4, 8):
            small.append((xc + k - 1.0, xc + k + 1.0, -34.6, -28.0))
    groups = {'A': ['MCU1', '5V1', '3V3', 'STEP_EN1'],
              'B': ['DC_Fault1', 'DC_Fault2', 'DC_Fault3', 'DC_Fault4'],
              'C': ['VCC1']}
    bbs = {}
    for c in des.allComponents:
        for mb in c.meshBodies:
            if mb.name in sum(groups.values(), []):
                b = mb.boundingBox
                bbs[mb.name] = [b.minPoint.x * 10, b.minPoint.y * 10, b.maxPoint.x * 10, b.maxPoint.y * 10]
    for g, names in groups.items():
        r = [bbs[n] for n in names if n in bbs]
        x0 = min(v[0] for v in r); y0 = min(v[1] for v in r)
        x1 = max(v[2] for v in r); y1 = max(v[3] for v in r)
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        hx, hy = max((x1 - x0) / 2.0 + 0.8, 1.5), max((y1 - y0) / 2.0 + 0.8, 1.5)
        small.append((cx - hx, cx + hx, cy - hy, cy + hy))

    for (x0, x1, y0, y1) in big:
        D(lid, rbox(x0, x1, y0, y1, BR, RZ0, RZ1))
    for (x0, x1, y0, y1) in small:
        D(lid, box(x0, x1, y0, y1, RZ0, RZ1))
    D(lid, cyl(26.5, -23.07, RZ0, RZ1, 3.25))                                 # lỗ chỉnh biến trở RV1
    for (x0, x1, y0, y1) in wall:
        D(lid, box(x0, x1, y0, y1, -1.0, WZ1))
        D(base, box(x0, x1, y0, y1, 0.0, WZ1))
    D(lid, box(-0.3, 13.7, 47.0, 54.0, -1.0, 6.4))                            # USB-C
    D(base, box(-0.3, 13.7, 47.0, 54.0, -1.0, 6.4))

    # ---- tạo component ----
    grp = root.occurrences.addNewComponent(A.Matrix3D.create())
    grp.component.name = '3D_Printing_Components'
    top_occ = grp.component.occurrences.addNewComponent(A.Matrix3D.create())
    top_occ.component.name = 'Enclosure_Top'
    bot_occ = grp.component.occurrences.addNewComponent(A.Matrix3D.create())
    bot_occ.component.name = 'Enclosure_Bottom'
    tb = top_occ.component.bRepBodies.add(lid); tb.name = 'Enclosure_Top'
    bb = bot_occ.component.bRepBodies.add(base); bb.name = 'Enclosure_Bottom'
    lib = app.materialLibraries.itemByName('Fusion Appearance Library')
    for body, nm in ((tb, 'Plastic - Translucent Matte (Green)'), (bb, 'Plastic - Translucent Matte (Red)')):
        ap = lib.appearances.itemByName(nm)
        try:
            body.appearance = ap
        except Exception:
            body.appearance = des.appearances.addByCopy(ap, nm)

    # Không vát, không bo mép nào ở nắp: mép sắc in đẹp hơn (xem skill mcu-design, mục 6b)
    tbody = top_occ.component.bRepBodies.item(0)

    print(json.dumps({'top': [tbody.isSolid, tbody.faces.count], 'bottom': [bb.isSolid, bb.faces.count]}))
