#!/usr/bin/env python3
"""Obstacle layout for runs without camera: a small web page on the Jetson.

    python3 ~/ros2_ws/src/sim_obstacles_gui.py          (or: simgui)
    -> open http://<jetson>:8780 in the browser (Mac, phone, ...)
       (not 8765: that is the foxglove_bridge)

Click seats to place red / green pylons, roll random layouts, mark pylons that
only appear late (when the robot is X cm away -- like a pylon the camera sees
late). "Speichern" writes config/sim_obstacles.txt. The controller reads it
with

    ros2 run ekf round1_controller ... -p sim_obstacles:=file

(it passes it to the scan_processor, which places the pylons on the seat grid
after the start detection -- see sim_obstacles in scan_processor_node.py).

Without a browser:
    python3 sim_obstacles_gui.py --random [--seed N] [--late 30]
        writes a random layout and prints it.

Only the Python standard library -- runs on the Jetson host, no container.
"""
import argparse
import json
import os
import random
import re
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WS = os.environ.get('ROBOT_WS', os.path.expanduser('~/ros2_ws'))
LAYOUT_FILE = os.path.join(WS, 'config', 'sim_obstacles.txt')
ENTRY_RE = re.compile(r'^s[0-3]:(entry|middle|exit):(inner|outer):(red|green)(:r\d{1,3})?$')
ROWS = ('entry', 'middle', 'exit')


def random_layout(rng, n_min=1, n_max=2, bay=True, late_pct=0, late_min=40, late_max=100):
    """Random layout: per straight n_min..n_max pylons, never two in one row;
    with a parking bay only the inner column on the start straight (rules)."""
    out = []
    for k in range(4):
        rows = rng.sample(ROWS, rng.randint(n_min, n_max))
        for row in rows:
            col = 'inner' if (bay and k == 0) else rng.choice(('inner', 'outer'))
            e = 's%d:%s:%s:%s' % (k, row, col, rng.choice(('red', 'green')))
            if late_pct > 0 and rng.random() * 100 < late_pct:
                e += ':r%d' % rng.randint(late_min, late_max)
            out.append(e)
    return out


def read_layout():
    try:
        with open(LAYOUT_FILE) as f:
            return [ln.split('#')[0].strip() for ln in f if ln.split('#')[0].strip()]
    except OSError:
        return []


def write_layout(entries):
    bad = [e for e in entries if not ENTRY_RE.match(e)]
    if bad:
        raise ValueError('not understood: ' + ', '.join(bad))
    os.makedirs(os.path.dirname(LAYOUT_FILE), exist_ok=True)
    tmp = LAYOUT_FILE + '.tmp'
    with open(tmp, 'w') as f:
        f.write('# simulated pylons, written by sim_obstacles_gui.py\n')
        f.write('# s<straight 0=start>:<entry|middle|exit>:<inner|outer>:<red|green>[:r<cm> appears late]\n')
        for e in entries:
            f.write(e + '\n')
    os.replace(tmp, LAYOUT_FILE)


PAGE = r"""<!doctype html>
<html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Hindernis-Simulator</title>
<style>
:root{--bg:#f6f6f4;--panel:#fff;--ink:#1d1d1b;--mute:#6b6b66;--line:#d8d8d2;--field:#fbfbf9;
--wall:#2b2b29;--red:#d6453d;--green:#2f9e5b;--mag:#c03ac0;--accent:#2f6fd6}
@media (prefers-color-scheme:dark){:root{--bg:#161615;--panel:#1f1f1d;--ink:#ecece8;--mute:#9a9a94;
--line:#3a3a36;--field:#232321;--wall:#d8d8d2;--accent:#6a9cf0}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);
font:15px/1.4 system-ui,-apple-system,Segoe UI,sans-serif}
main{max-width:1100px;margin:0 auto;padding:16px;display:grid;gap:16px;grid-template-columns:minmax(0,1fr) 320px}
@media (max-width:860px){main{grid-template-columns:1fr}}
h1{font-size:18px;margin:0 0 4px}.sub{color:var(--mute);font-size:13px;margin:0}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px}
svg{width:100%;height:auto;display:block;touch-action:manipulation}
.seat{cursor:pointer}.seat:hover circle.base{stroke:var(--accent);stroke-width:0.022}
fieldset{border:0;padding:0;margin:0 0 12px}legend{font-weight:600;font-size:13px;margin-bottom:6px}
.seg{display:flex;gap:6px;flex-wrap:wrap}.seg button{flex:1}
button{font:inherit;padding:7px 10px;border-radius:8px;border:1px solid var(--line);
background:var(--bg);color:var(--ink);cursor:pointer}button.on{border-color:var(--accent);
box-shadow:inset 0 0 0 1px var(--accent);font-weight:600}button.primary{background:var(--accent);
color:#fff;border-color:var(--accent)}
label{display:flex;justify-content:space-between;align-items:center;gap:8px;font-size:14px;margin:4px 0}
input[type=number]{width:70px;font:inherit;padding:4px 6px;border-radius:6px;border:1px solid var(--line);
background:var(--bg);color:var(--ink)}
textarea{width:100%;min-height:110px;font:13px ui-monospace,Menlo,monospace;border-radius:8px;
border:1px solid var(--line);background:var(--bg);color:var(--ink);padding:8px}
code{font:12px ui-monospace,Menlo,monospace;word-break:break-all;color:var(--mute)}
#status{font-size:13px;min-height:18px;margin-top:6px}.ok{color:var(--green)}.err{color:var(--red)}
.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:12px;color:var(--mute);margin-top:8px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;vertical-align:-1px;margin-right:4px}
</style></head><body><main>
<section class="card">
  <h1>Hindernis-Simulator</h1>
  <p class="sub">Platz anklicken mit dem gewählten Werkzeug. s0 = Start-/Zielgerade (unten, mit Parkbucht).</p>
  <svg id="field" viewBox="-1.75 -1.75 3.5 3.5"></svg>
  <div class="legend">
    <span><span class="dot" style="background:var(--red)"></span>rot (rechts vorbei)</span>
    <span><span class="dot" style="background:var(--green)"></span>grün (links vorbei)</span>
    <span>gestrichelt + Zahl = taucht erst bei X cm Abstand auf</span>
    <span>E / M / A = Einfahrt / Mitte / Ausfahrt der Geraden</span>
  </div>
</section>
<aside class="card">
  <fieldset><legend>Fahrtrichtung</legend><div class="seg">
    <button data-dir="CW">CW (im Uhrzeigersinn)</button><button data-dir="CCW">CCW</button></div></fieldset>
  <fieldset><legend>Werkzeug</legend><div class="seg">
    <button data-tool="red">Rot</button><button data-tool="green">Grün</button>
    <button data-tool="late">Spät</button><button data-tool="del">Löschen</button></div>
    <label>Spät erscheinen bei (cm)<input id="lateDist" type="number" min="20" max="300" value="80"></label>
  </fieldset>
  <fieldset><legend>Zufall</legend>
    <label>Pylone pro Gerade min<input id="nMin" type="number" min="0" max="3" value="1"></label>
    <label>Pylone pro Gerade max<input id="nMax" type="number" min="0" max="3" value="2"></label>
    <label>davon spät (%)<input id="latePct" type="number" min="0" max="100" value="0"></label>
    <label>spät ab cm (min/max)<span><input id="lateMin" type="number" value="40"> <input id="lateMax" type="number" value="100"></span></label>
    <label>Parkbucht (s0 nur innen)<input id="bay" type="checkbox" checked></label>
    <div class="seg"><button id="rand">Zufall würfeln</button><button id="clear">Leeren</button></div>
  </fieldset>
  <fieldset><legend>Aufstellung</legend>
    <textarea id="spec" spellcheck="false"></textarea>
    <div class="seg" style="margin-top:8px"><button id="load">Laden</button><button id="save" class="primary">Speichern</button></div>
    <div class="seg" style="margin-top:6px"><button id="randSave" class="primary">&#127922; Zufall + Speichern</button></div>
    <div id="status"></div>
  </fieldset>
  <fieldset><legend>Start</legend>
    <code>ros2 run ekf round1_controller ... -p sim_obstacles:=file</code>
  </fieldset>
</aside></main>
<script>
const NS='http://www.w3.org/2000/svg', ROWS=['entry','middle','exit'], RL={entry:'E',middle:'M',exit:'A'};
let dir='CW', tool='red', layout={};   // key "k:row:col" -> {color, reveal}
const $=id=>document.getElementById(id);
function el(t,a,p){const e=document.createElementNS(NS,t);for(const k in a)e.setAttribute(k,a[k]);(p||svg).appendChild(e);return e}
const svg=$('field');
// geometry: straight k sits at angle base + k*step; s0 at the bottom.
function frame(k){const step=dir==='CCW'?90:-90, a=(-90+k*step)*Math.PI/180;
  const u=[Math.cos(a),Math.sin(a)], t=dir==='CCW'?[-u[1],u[0]]:[u[1],-u[0]];return{u,t}}
function seatPos(k,row,col){const {u,t}=frame(k), r=col==='inner'?0.9:1.1, s={entry:-0.5,middle:0,exit:0.5}[row];
  return [u[0]*r+t[0]*s, -(u[1]*r+t[1]*s)]}   // svg y down
function draw(){svg.innerHTML='';
  el('rect',{x:-1.5,y:-1.5,width:3,height:3,fill:'var(--field)',stroke:'var(--wall)','stroke-width':0.03});
  el('rect',{x:-0.5,y:-0.5,width:1,height:1,fill:'var(--bg)',stroke:'var(--wall)','stroke-width':0.03});
  // parking bay on the start straight (approximate position)
  // CW: front 1.92 m, rear axle at x=+0.42 driving -x; CCW: front 1.16 m, x=+0.34 driving +x
  const bx=dir==='CW'?[0.22,0.48]:[0.28,0.54];
  for(const x of bx) el('rect',{x:x-0.01,y:1.3,width:0.02,height:0.2,fill:'var(--mag)'});
  for(let k=0;k<4;k++){const {u,t}=frame(k);
    const c=[u[0]*1.0,-u[1]*1.0], ah=[c[0]+t[0]*0.32,c[1]-t[1]*0.32], at=[c[0]-t[0]*0.32,c[1]+t[1]*0.32];
    el('line',{x1:at[0],y1:at[1],x2:ah[0],y2:ah[1],stroke:'var(--line)','stroke-width':0.025});
    el('polygon',{points:`${ah[0]},${ah[1]} ${ah[0]-t[0]*0.08+t[1]*0.05},${ah[1]+t[1]*0.08+t[0]*0.05} ${ah[0]-t[0]*0.08-t[1]*0.05},${ah[1]+t[1]*0.08-t[0]*0.05}`,fill:'var(--line)'});
    const lp=[u[0]*1.62,-u[1]*1.62];
    const lab=el('text',{x:lp[0],y:lp[1]+0.04,'font-size':0.11,'text-anchor':'middle',fill:'var(--mute)'});lab.textContent='s'+k;
    for(const row of ROWS){const p=seatPos(k,row,'outer'), q=[p[0]+u[0]*0.14,p[1]-u[1]*0.14];
      const tl=el('text',{x:q[0],y:q[1]+0.03,'font-size':0.08,'text-anchor':'middle',fill:'var(--mute)'});tl.textContent=RL[row];
      for(const col of ['inner','outer']){const [x,y]=seatPos(k,row,col), key=`${k}:${row}:${col}`;
        const locked=k===0&&col==='outer'&&$('bay').checked;
        const g=el('g',{class:'seat'});g.onclick=()=>click(key,locked);
        const o=layout[key];
        el('circle',{class:'base',cx:x,cy:y,r:0.065,fill:o?`var(--${o.color})`:'var(--panel)',
          stroke:locked?'var(--line)':'var(--mute)','stroke-width':0.012,'stroke-dasharray':o&&o.reveal?'0.03 0.02':'',opacity:locked?0.35:1},g);
        if(o&&o.reveal){el('circle',{cx:x,cy:y,r:0.09,fill:'none',stroke:`var(--${o.color})`,'stroke-width':0.012,'stroke-dasharray':'0.03 0.02'},g);
          const tx=el('text',{x:x,y:y+0.028,'font-size':0.075,'text-anchor':'middle',fill:'#fff','font-weight':700},g);tx.textContent=o.reveal}
      }}}
  // robot in the bay
  const rx=dir==='CW'?0.37:0.39, d=dir==='CW'?-1:1;
  el('rect',{x:rx-0.09,y:1.33,width:0.18,height:0.11,rx:0.02,fill:'var(--accent)',opacity:0.8});
  el('polygon',{points:`${rx+d*0.12},1.385 ${rx+d*0.05},1.35 ${rx+d*0.05},1.42`,fill:'var(--accent)'});
  $('spec').value=toSpec().join('\n')}
function click(key,locked){
  if(tool==='del'){delete layout[key]}
  else if(tool==='late'){const o=layout[key];if(!o){status('Erst einen Pylon setzen, dann mit "Spät" markieren.','err');return}
    const d=Math.min(300,Math.max(20,+$('lateDist').value||80));o.reveal=o.reveal===d?0:d}
  else{if(locked){status('Mit Parkbucht ist auf s0 nur die innere Spalte erlaubt.','err');return}
    const [k,row]=key.split(':');for(const c of['inner','outer'])if(`${k}:${row}:${c}`!==key)delete layout[`${k}:${row}:${c}`];
    layout[key]={color:tool,reveal:(layout[key]||{}).reveal||0}}
  draw()}
function toSpec(){return Object.keys(layout).sort().map(k=>{const [s,row,col]=k.split(':'),o=layout[k];
  return `s${s}:${row}:${col}:${o.color}`+(o.reveal?`:r${o.reveal}`:'')})}
function fromSpec(lines){layout={};const bad=[];for(const ln of lines){if(!ln.trim())continue;const m=ln.trim().match(/^s([0-3]):(entry|middle|exit):(inner|outer):(red|green)(?::r(\d+))?$/);
  if(m)layout[`${m[1]}:${m[2]}:${m[3]}`]={color:m[4],reveal:m[5]?+m[5]:0};else bad.push(ln.trim())}draw();
  if(bad.length)status('Nicht verstanden: '+bad.join(', '),'err')}
function status(t,c){const s=$('status');s.textContent=t;s.className=c||''}
function randomize(){layout={};const nmin=+$('nMin').value,nmax=Math.max(nmin,+$('nMax').value),pct=+$('latePct').value,
  lmin=+$('lateMin').value,lmax=Math.max(lmin,+$('lateMax').value);
  for(let k=0;k<4;k++){const n=nmin+Math.floor(Math.random()*(nmax-nmin+1)),rows=[...ROWS].sort(()=>Math.random()-0.5).slice(0,n);
    for(const row of rows){const col=(k===0&&$('bay').checked)?'inner':(Math.random()<0.5?'inner':'outer');
      layout[`${k}:${row}:${col}`]={color:Math.random()<0.5?'red':'green',reveal:Math.random()*100<pct?lmin+Math.floor(Math.random()*(lmax-lmin+1)):0}}}
  draw();status('Gewürfelt -- noch nicht gespeichert.')}
async function load(){const r=await fetch('api/layout');const j=await r.json();fromSpec(j.entries);status(j.entries.length?`Geladen: ${j.entries.length} Pylone`:'Datei leer / nicht vorhanden')}
async function save(){const entries=$('spec').value.split(/[\n+]/).map(s=>s.trim()).filter(Boolean);
  const r=await fetch('api/layout',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({entries})});
  const j=await r.json();if(r.ok){fromSpec(entries);status(`Gespeichert (${entries.length} Pylone) -- gilt ab dem nächsten Controller-Start.`,'ok')}else status(j.error,'err')}
document.querySelectorAll('[data-dir]').forEach(b=>b.onclick=()=>{dir=b.dataset.dir;sync();draw()});
document.querySelectorAll('[data-tool]').forEach(b=>b.onclick=()=>{tool=b.dataset.tool;sync()});
function sync(){document.querySelectorAll('[data-dir]').forEach(b=>b.classList.toggle('on',b.dataset.dir===dir));
  document.querySelectorAll('[data-tool]').forEach(b=>b.classList.toggle('on',b.dataset.tool===tool))}
$('rand').onclick=randomize;$('clear').onclick=()=>{layout={};draw()};$('save').onclick=save;$('load').onclick=load;
$('bay').onchange=()=>{if($('bay').checked){const n=['entry','middle','exit'].filter(r=>layout[`0:${r}:outer`]).length;
  for(const r of ROWS)delete layout[`0:${r}:outer`];if(n)status(`${n} Pylon(e) außen auf s0 entfernt (Parkbucht).`)}draw()};
$('randSave').onclick=async()=>{randomize();await save()};$('spec').onchange=()=>fromSpec($('spec').value.split(/[\n+]/));
sync();draw();load();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype='application/json'):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype + '; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._send(200, PAGE, 'text/html')
        elif self.path == '/api/layout':
            self._send(200, json.dumps({'entries': read_layout(), 'file': LAYOUT_FILE}))
        else:
            self._send(404, json.dumps({'error': 'not found'}))

    def do_POST(self):
        if self.path != '/api/layout':
            self._send(404, json.dumps({'error': 'not found'}))
            return
        try:
            n = int(self.headers.get('Content-Length', 0))
            entries = [str(e).strip() for e in json.loads(self.rfile.read(n))['entries']]
            write_layout(entries)
            print('saved: ' + ' + '.join(entries) if entries else 'saved: (empty)', flush=True)
            self._send(200, json.dumps({'ok': True}))
        except (ValueError, KeyError, TypeError) as err:
            self._send(400, json.dumps({'error': str(err)}))

    def log_message(self, *_):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=8780)   # 8765 = foxglove_bridge
    ap.add_argument('--random', action='store_true', help='write a random layout and exit')
    ap.add_argument('--seed', type=int)
    ap.add_argument('--min', type=int, default=1, help='pylons per straight, min')
    ap.add_argument('--max', type=int, default=2, help='pylons per straight, max')
    ap.add_argument('--late', type=int, default=0, help='percent of pylons that appear late')
    ap.add_argument('--no-bay', action='store_true', help='no parking bay: outer column on s0 allowed')
    a = ap.parse_args()
    if a.random:
        entries = random_layout(random.Random(a.seed), a.min, a.max, not a.no_bay, a.late)
        write_layout(entries)
        print('\n'.join(entries))
        print('-> %s' % LAYOUT_FILE)
        return
    ThreadingHTTPServer.allow_reuse_address = True
    try:
        srv = ThreadingHTTPServer(('0.0.0.0', a.port), Handler)
    except OSError as err:
        print('Port %d not free (%s) -- already running, or another program '
              '(8765 = foxglove_bridge). Other port: simgui --port 8781' % (a.port, err))
        return 1
    host = os.uname().nodename
    print('Obstacle GUI: http://%s:%d  (file %s) -- Ctrl-C to stop' % (host, a.port, LAYOUT_FILE), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main())
