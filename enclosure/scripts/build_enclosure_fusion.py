"""
Dựng vỏ 2 mảnh (Enclosure_Top, Enclosure_Bottom) cho board MM_Control trong Autodesk Fusion.

Cách dùng: mở file MM_Control_Board_v7.f3d (có mô hình board dạng mesh), chạy script này
trong Fusion (Utilities > Scripts and Add-Ins) hoặc qua Fusion MCP. Script xóa component
'3D_Printing_Components' cũ rồi dựng lại, nên chạy lại được.

Vị trí LED, terminal, đầu XH lấy từ MM_Control.kicad_pcb (x = x_kicad - 100, y = 100 - y_kicad),
không đọc mesh nên chạy được trong file Fusion bất kỳ.

Đơn vị trong script: mm (đổi sang cm khi gọi API Fusion). Trục Z hướng lên, PCB nằm z = DZ .. DZ + 1,6 mm
(DZ: chân đỡ nâng board lên), tâm board ở gốc tọa độ, board 100 x 100 mm, lỗ vít Ø4,2 mm tại (±45, ±45).

Quy ước cắt cửa: hai ô cắt cách nhau dưới 10 mm thì gộp thành một ô, không chừa vách (vách mỏng dễ gãy).
Khe thông gió 2 mm là ngoại lệ vì cố ý chừa sườn.
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
    DZ = 1.0                      # chân đỡ cao thêm: thân XH nằm ngang chìa qua mép board, phải nâng board khỏi gờ định vị
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
            U(base, cyl(cx, cy, ZF - 0.5, DZ, POST_R))                     # standoff
            U(base, box(min(cx, sx * FILL), max(cx, sx * FILL), min(cy, sy * FILL), max(cy, sy * FILL), ZF - 0.5, DZ))
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * HC, sy * HC
            D(base, cyl(cx, cy, ZB - 1.0, DZ + 1.0, SCREW_HOLE))
            D(base, cyl(cx, cy, ZB - 1.0, ZB + CB_D, CB_R))

    # ---- nắp ----
    lid = rrect(OH, RO, 0.0, ZT)
    D(lid, rrect(CH, RC, -1.0, ZR))
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * HC, sy * HC
            U(lid, cyl(cx, cy, 1.6 + DZ, ZR + 0.5, POST_R))
            U(lid, box(min(cx, sx * FILL), max(cx, sx * FILL), min(cy, sy * FILL), max(cy, sy * FILL), 1.6 + DZ, ZR + 0.5))
    for sx in (-1, 1):
        for sy in (-1, 1):
            D(lid, cyl(sx * HC, sy * HC, 1.0, 1.6 + DZ + INS_DEPTH, INS_R))
    D(lid, D(rrect(51.8, RC + 1.3, -1.0, 3.3), rrect(CH, RC, -2.0, 4.3)))   # rãnh khớp với gờ của đế

    # ---- cutout ----
    RZ0, RZ1 = ZR - 1.0, ZT + 1.0
    WZ1 = 11.5 + DZ
    big = []     # khoang tay thao tác trên mái: bo góc R2
    small = []   # khe thông gió, cửa sổ LED: không bo
    wall = []    # cửa sổ ở vách (cắt cả nắp và đế)
    # Terminal bên +X: hai terminal trên cách nhau 4,4 mm nên gộp một ô; terminal dưới gộp thêm LED VCC1 (48,09; -27,48)
    wall.append((48.0, 54.0, 11.82, 38.98))
    wall.append((48.0, 54.0, -24.22, -12.32))
    big.append((39.5, 48.5, 12.02, 38.78))
    big.append((39.5, 49.0, -29.7, -12.52))
    wall.append((-54.0, -48.0, 7.35, 34.48))                                  # 2 terminal bên -X, gộp một ô
    wall.append((-18.2, 13.7, 48.0, 54.0))                                    # CAN (J17) + USB-C, cách 6,4 mm nên gộp
    # Đầu XH stepper và servo cắm ngang từ phía -Y; các cửa cách nhau 1,4 / 6,7 / 7,3 mm nên gộp một cửa.
    # XH: thân 12,4 mm, tâm x = -17,25; 3,88; 25,6. Servo SPOX 3 chân: tâm x = -31,8 (đoán từ footprint), thân 9,9 mm. Mỗi bên chừa 1 mm
    wall.append((-37.7, 32.8, -54.0, -48.0))
    big.append((-50.5, -28.0, -24.0, 34.5))                                   # khoang header SPI/I2C/UART và terminal -X
    big.append((-37.5, -7.2, 38.0, 50.5))                                     # SWD + CAN, gộp
    big.append((16.5, 37.0, 38.0, 50.5))                                      # BOOT0 / RESET
    # khe thông gió căn giữa 3 tản nhiệt TMC2209 (tản 9 x 9 mm, tâm y = -33,78): 3 khe 2 x 7 mm, bước 4 mm
    for xc in (25.6, 3.88, -17.25):
        for k in (-4, 0, 4):
            small.append((xc + k - 1.0, xc + k + 1.0, -33.78 - 3.5, -33.78 + 3.5))
    # cửa sổ LED: toạ độ LED từ PCB, chừa 2,2 mm quanh tâm LED ngoài cùng
    leds = {'status': [(-15.55, -14.95), (-15.50, -16.57), (-15.50, -18.20), (-15.53, -19.83)],   # STEP_EN1, MCU1, 3V3, 5V1
            'fault': [(13.18, 21.53), (13.18, 23.16), (13.18, 24.78), (13.18, 26.41)]}              # DC_Fault4..1
    for pts in leds.values():
        small.append((min(p[0] for p in pts) - 2.2, max(p[0] for p in pts) + 2.2,
                      min(p[1] for p in pts) - 2.2, max(p[1] for p in pts) + 2.2))

    for (x0, x1, y0, y1) in big:
        D(lid, rbox(x0, x1, y0, y1, BR, RZ0, RZ1))
    for (x0, x1, y0, y1) in small:
        D(lid, box(x0, x1, y0, y1, RZ0, RZ1))
    D(lid, cyl(26.5, -23.07, RZ0, RZ1, 3.25))                                 # lỗ chỉnh biến trở RV1
    for (x0, x1, y0, y1) in wall:
        D(lid, box(x0, x1, y0, y1, -1.0, WZ1))
        D(base, box(x0, x1, y0, y1, 0.0, WZ1))

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
