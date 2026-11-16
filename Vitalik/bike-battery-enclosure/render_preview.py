"""Рендер превью сборки в out/preview*.png.

three.js в headless Chromium (Playwright для Node.js: `npm i -g playwright`). three.js берётся с CDN; если CDN
недоступен -- задайте THREE_DIR=<папка распакованного npm-пакета three@0.160.0>
(получить: `npm pack three@0.160.0 && tar xzf three-0.160.0.tgz` -> ./package).
"""

import base64
import json
import os
import pathlib
import subprocess
import tempfile

import cadquery as cq

import enclosure as e

CDN = "https://cdn.jsdelivr.net/npm/three@0.160.0/"

HTML = r"""<!doctype html><html><body style="margin:0;background:#fff">
<script type="importmap">{"imports":{"three":"%CDN%build/three.module.js","three/addons/":"%CDN%examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {toCreasedNormals} from 'three/addons/utils/BufferGeometryUtils.js';
const P = window.SCENE;
const r = new THREE.WebGLRenderer({antialias:true, preserveDrawingBuffer:true});
r.setPixelRatio(P.dpr); r.setSize(P.w, P.h); r.setClearColor(0xffffff); document.body.appendChild(r.domElement);
const s = new THREE.Scene();
s.add(new THREE.HemisphereLight(0xffffff, 0x8890a0, 1.5));
const d = new THREE.DirectionalLight(0xffffff, 1.7); d.position.set(-0.6, -1, 1.5); s.add(d);
const d2 = new THREE.DirectionalLight(0xffffff, 0.5); d2.position.set(1, 0.6, -0.4); s.add(d2);
const L = new STLLoader(); const box = new THREE.Box3();
for (const it of P.items) {
  const g = toCreasedNormals(L.parse(Uint8Array.from(atob(it.stl), c => c.charCodeAt(0)).buffer), Math.PI / 5);
  const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({color: it.color, roughness: .65, metalness: .05,
      polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1}));
  m.position.set(...it.loc); s.add(m); box.expandByObject(m);
  const ed = new THREE.LineSegments(new THREE.EdgesGeometry(g, 40),
      new THREE.LineBasicMaterial({color: 0x1d1f23, transparent: true, opacity: .35}));
  ed.position.copy(m.position); s.add(ed);
}
const c = box.getCenter(new THREE.Vector3()), sz = box.getSize(new THREE.Vector3()).length();
const cam = new THREE.OrthographicCamera(-1, 1, 1, -1, -5000, 5000);
cam.up.set(0, 0, 1);
cam.position.copy(c.clone().add(new THREE.Vector3(...P.dir).normalize().multiplyScalar(1500)));
cam.lookAt(c);
const k = sz * P.zoom, a = P.w / P.h;
cam.left = -k * a; cam.right = k * a; cam.top = k; cam.bottom = -k; cam.updateProjectionMatrix();
r.render(s, cam); window.DONE = true;
</script></body></html>""".replace("%CDN%", CDN)


def stl_b64(shape, tmp) -> str:
    p = pathlib.Path(tmp) / "x.stl"
    cq.exporters.export(shape, str(p), tolerance=0.1, angularTolerance=0.2)
    return base64.b64encode(p.read_bytes()).decode()


_CACHE = {}


def model():
    if not _CACHE:
        parts, _ = e.build()
        _CACHE.update(parts=parts, latches=e.placed_latches(), bat=e.dummy_battery(),
                      ctrl=e.dummy_controller())
    return _CACHE


def scene(ex, tmp, cutaway=False, crop=None):
    m = model()
    parts = m["parts"]
    bat, bms = m["bat"]
    tube = cq.Workplane("YZ").circle(17).extrude(240, both=True).translate((0, 0, e.H + 17))
    body = parts["body"]
    if cutaway:  # убираем ближнюю (-Y) половину корпуса, чтобы видеть содержимое
        body = body.cut(cq.Workplane("XY").box(600, 200, 400).translate((0, -100, 150)))
    items = [
        (body, (0, 0, 0), "#a3abb5"),
        (parts["lid"].translate((0, 0, e.BASE_H)), (0, 0, ex * 2.4), "#3a3d42"),
    ]
    latches = m["latches"]
    if cutaway:  # защёлки убранной ближней стенки не показываем
        latches = [l for l in latches if l.val().Center().y > 0]
    items += [(l, (0, 0, 0), "#e0902a") for l in latches]
    if not crop:
        items.append((tube, (0, 0, ex * 3.2), "#d2d5da"))
    if ex or cutaway:
        items += [
            (parts["shelf"].translate((0, 0, e.SHELF_Z)), (0, 0, ex * 0.9), "#6b7078"),
            (m["ctrl"], (0, 0, ex * 0.3), "#c4581e"),
            (bat, (0, 0, ex * 1.5), "#2f7bd1"),
            (bms, (0, -ex * 0.8, ex * 1.5), "#2e9e4a"),
        ]
    if crop:
        box = cq.Workplane("XY").box(*crop[1]).translate(crop[0])
        items = [(sh.intersect(box), loc, col) for sh, loc, col in items]
        items = [it for it in items if it[0].solids().size()]
    return [{"stl": stl_b64(sh, tmp), "loc": list(loc), "color": col} for sh, loc, col in items]


NODE_JS = r"""
const {chromium} = require('playwright');
const fs = require('fs');
const [htmlPath, pngPath, w, h, cdn, threeDir] = process.argv.slice(1);
(async () => {
  const b = await chromium.launch({args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader']});
  const pg = await b.newPage({viewport: {width: +w, height: +h}});
  if (threeDir) await pg.route(cdn + '**', r => r.fulfill({
      path: threeDir + '/' + r.request().url().slice(cdn.length), contentType: 'text/javascript'}));
  await pg.setContent(fs.readFileSync(htmlPath, 'utf8'));
  await pg.waitForFunction('window.DONE === true', null, {timeout: 180000});
  await pg.screenshot({path: pngPath});
  await b.close();
})().catch(e => { console.error(e); process.exit(1); });
"""


def render(items, direction, path, tmp, zoom=0.4, w=1200, h=800):
    scene_js = json.dumps({"items": items, "dir": direction, "w": w, "h": h, "dpr": 2, "zoom": zoom})
    html = HTML.replace("<body", f"<script>window.SCENE = {scene_js};</script><body", 1)
    hp = pathlib.Path(tmp) / "scene.html"
    hp.write_text(html, encoding="utf-8")
    subprocess.run(["node", "-e", NODE_JS, str(hp), str(path), str(w), str(h), CDN,
                    os.environ.get("THREE_DIR", "")], check=True)
    print("saved", path)


if __name__ == "__main__":
    e.OUT.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        render(scene(0, tmp), [-0.6, -1, 0.45], e.OUT / "preview.png", tmp)
        render(scene(55, tmp), [-0.6, -1, 0.55], e.OUT / "preview_exploded.png", tmp, zoom=0.42)
        render(scene(0, tmp, cutaway=True), [0.12, -1, 0.12], e.OUT / "preview_cutaway.png", tmp)
        render(scene(0, tmp), [0.9, -0.5, -0.6], e.OUT / "preview_bottom.png", tmp)
        render(scene(0, tmp), [1, -0.15, 0.25], e.OUT / "preview_rear.png", tmp, zoom=0.36)
        x = e.LATCH_X[1]
        crop = ((x, -e.W / 2, e.BASE_H - 15), (70, 40, 60))
        render(scene(0, tmp, crop=crop), [-0.5, -1, 0.35], e.OUT / "preview_latch.png", tmp, zoom=0.5)
