#!/usr/bin/env python3
"""
PuzzleBot Real-Time Dashboard v3
─────────────────────────────────
localhost:8080

Charts (4x3 grid):
 Row 1: Lyapunov V(t) | XY Trajectory (click to draw waypoints) | Phase (e_d,ė_d) + sliding surface s_v=0
 Row 2: Distance error | Control signals v,ω | Phase (θ_e,θ̇_e) + sliding surface s_w=0
 Row 3: V̇(t) derivative | Sliding surfaces s_v(t),s_w(t) | Phase (v,ω) velocity space
 Row 4: Heading error | Perturbations | Sliding phase (s_v,ṡ_v) convergence to origin

Full history — no truncation. Sliding surface lines overlaid on phase plots.
Custom trajectories: click XY chart or use preset buttons.
Reset: calls /reset_simulation to reset robot pose.
"""
import rclpy, numpy as np, json, threading, time, urllib.parse
from http.server import HTTPServer, BaseHTTPRequestHandler
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String, Bool
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Vector3, PoseStamped
from tf_transformations import euler_from_quaternion
from collections import deque

MAX_PTS = 10000  # full history

class DataStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.keys = ['t','V','dist','angle_err','dist_dot','angle_dot',
                     'v','w','x','y','theta','pv','pw','s_v','s_w']
        self.buf = {k: deque(maxlen=MAX_PTS) for k in self.keys}
        self.ctrl='PID'; self.arrived=False
        self.gx=2.0; self.gy=1.5; self.wp=0; self.wp_total=0
        self.waypoints=[]; self.t0=time.time()

    def push(self, d):
        with self.lock:
            now=time.time()-self.t0
            self.buf['t'].append(now)
            for k in self.keys[1:]:
                val = d.get(k, 0.0)
                if k=='dist': val = d.get('dist_err', d.get('dist', 0.0))
                self.buf[k].append(float(val))
            self.ctrl=d.get('ctrl',self.ctrl)
            self.gx=d.get('gx',self.gx); self.gy=d.get('gy',self.gy)
            self.wp=d.get('wp',0); self.wp_total=d.get('wp_total',0)

    def push_perturb(self, pv, pw):
        with self.lock:
            self.buf['pv'].append(pv); self.buf['pw'].append(pw)

    def reset(self):
        with self.lock:
            for d in self.buf.values(): d.clear()
            self.t0=time.time(); self.waypoints=[]

    def snapshot(self):
        """Return latest data downsampled to at most 400 points using block averaging.

        Naive stride sampling (arr[::step]) aliases high-frequency signals into
        apparent noise as the buffer fills. Block averaging (mean of each block)
        acts as a proper anti-aliasing low-pass filter so the charts stay smooth
        regardless of how long the run has been going.
        """
        with self.lock:
            n = len(self.buf['t'])
            target = 400
            r = {}
            if n <= target:
                # Not enough points yet — send everything as-is
                for k, v in self.buf.items():
                    r[k] = list(v)
            else:
                # Block-average: split each series into `target` equal chunks,
                # take the mean of each chunk. Preserves envelope, kills aliasing.
                arr_t = np.array(self.buf['t'])
                step = n / target  # float — use slice indices
                for k, v in self.buf.items():
                    arr = np.array(v, dtype=float)
                    # Build block means
                    out = []
                    for i in range(target):
                        lo = int(i * step)
                        hi = int((i + 1) * step)
                        if hi > lo:
                            out.append(float(arr[lo:hi].mean()))
                        elif lo < n:
                            out.append(float(arr[lo]))
                    r[k] = out
            r['ctrl']=self.ctrl; r['arrived']=self.arrived
            r['gx']=self.gx; r['gy']=self.gy
            r['wp']=self.wp; r['wp_total']=self.wp_total
            r['waypoints']=self.waypoints
            return r

store = DataStore()

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        p=self.path.split('?')[0]
        if p in ('/','/index.html'):
            self._resp(200,'text/html',HTML.encode())
        elif p=='/stream':
            self.send_response(200)
            self.send_header('Content-Type','text/event-stream')
            self.send_header('Cache-Control','no-cache')
            self.send_header('Access-Control-Allow-Origin','*')
            self.end_headers()
            try:
                while True:
                    s=store.snapshot()
                    self.wfile.write(f'data: {json.dumps(s,default=float)}\n\n'.encode())
                    self.wfile.flush(); time.sleep(0.25)
            except: pass
        elif p.startswith('/api/switch'):
            q=urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            Handler._node.publish_switch(q.get('ctrl',['PID'])[0])
            self._resp(200,'text/plain',b'ok')
        elif p.startswith('/api/goal'):
            q=urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            Handler._node.publish_goal(float(q.get('x',['2'])[0]),float(q.get('y',['1.5'])[0]))
            self._resp(200,'text/plain',b'ok')
        elif p=='/api/reset':
            Handler._node.publish_reset(); store.reset()
            self._resp(200,'text/plain',b'ok')
        elif p.startswith('/api/trajectory'):
            q=urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            Handler._node.publish_trajectory(q.get('type',['circle'])[0],q.get('pts',['[]'])[0])
            self._resp(200,'text/plain',b'ok')
        else: self._resp(404,'text/plain',b'not found')

    def _resp(self, code, ct, body):
        self.send_response(code); self.send_header('Content-Type',ct); self.end_headers(); self.wfile.write(body)


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.declare_parameter('port', 8080)
        port=self.get_parameter('port').value
        self.create_subscription(String,'/controller_state',self._ctrl_cb,10)
        self.create_subscription(Odometry,'/odom',self._odom_cb,10)
        self.create_subscription(Vector3,'/terrain_perturbation',self._perturb_cb,10)
        self.create_subscription(Bool,'/arrived',self._arrived_cb,10)
        self.switch_pub=self.create_publisher(String,'/switch_controller',10)
        self.goal_pub=self.create_publisher(PoseStamped,'/goal_pose',10)
        self.reset_pub=self.create_publisher(String,'/sim_reset',10)
        self.path_pub=self.create_publisher(Path,'/trajectory',10)
        # Use /reset_simulation to reset robot pose + physics
        self.gz_reset=self.create_client(EmptySrv,'/reset_simulation')
        self.gz_reset_world=self.create_client(EmptySrv,'/reset_world')
        Handler._node=self
        self._srv=HTTPServer(('0.0.0.0',port),Handler)
        threading.Thread(target=self._srv.serve_forever,daemon=True).start()
        self.get_logger().info(f'Dashboard → http://localhost:{port}')

    def _ctrl_cb(self,msg):
        try: store.push(json.loads(msg.data))
        except: pass
    def _odom_cb(self,msg): pass
    def _perturb_cb(self,msg): store.push_perturb(msg.x,msg.z)
    def _arrived_cb(self,msg): store.arrived=msg.data

    def publish_switch(self,name): self.switch_pub.publish(String(data=name))
    def publish_goal(self,gx,gy):
        m=PoseStamped(); m.header.frame_id='odom'
        m.pose.position.x=float(gx); m.pose.position.y=float(gy)
        self.goal_pub.publish(m)

    def publish_reset(self):
        self.get_logger().info('Resetting...')
        # Try /reset_simulation first (resets pose + physics)
        if self.gz_reset.service_is_ready():
            self.gz_reset.call_async(EmptySrv.Request())
            self.get_logger().info('Called /reset_simulation')
        elif self.gz_reset_world.service_is_ready():
            self.gz_reset_world.call_async(EmptySrv.Request())
            self.get_logger().info('Called /reset_world')
        else:
            self.get_logger().warn('No Gazebo reset service available')
        # Reset controllers
        self.reset_pub.publish(String(data='{}'))
        store.reset()

    def publish_trajectory(self, ttype, pts_json):
        pts=[]
        if ttype=='circle':
            for a in np.linspace(0,2*np.pi,60): pts.append((1.0+0.8*np.cos(a),0.8+0.8*np.sin(a)))
        elif ttype=='figure8':
            for a in np.linspace(0,2*np.pi,80): pts.append((1.0+0.8*np.sin(a),0.8+0.5*np.sin(2*a)))
        elif ttype=='square':
            corners=[(0.3,0.3),(1.8,0.3),(1.8,1.5),(0.3,1.5),(0.3,0.3)]
            for i in range(len(corners)-1):
                x0,y0=corners[i]; x1,y1=corners[i+1]
                n=max(int(np.hypot(x1-x0,y1-y0)/0.1),3)
                for t in np.linspace(0,1,n,endpoint=False): pts.append((x0+t*(x1-x0),y0+t*(y1-y0)))
            pts.append(corners[-1])
        elif ttype=='zigzag':
            x=0.2
            for i in range(8): pts.append((x,0.4 if i%2==0 else 1.2)); x+=0.25
        elif ttype=='custom':
            try: pts=[(p[0],p[1]) for p in json.loads(pts_json)]
            except: pts=[(2.0,1.5)]
        else: pts=[(2.0,1.5)]
        store.waypoints=pts
        msg=Path(); msg.header.frame_id='odom'
        for px,py in pts:
            ps=PoseStamped(); ps.header.frame_id='odom'
            ps.pose.position.x=float(px); ps.pose.position.y=float(py)
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        self.get_logger().info(f'Trajectory: {ttype}, {len(pts)} pts')


# ═══════════════════════════════════════════════════════════════════════
HTML = r"""<!DOCTYPE html><html><head><meta charset="UTF-8">
<title>PuzzleBot Live Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#07070c;color:#ccc;font-family:'JetBrains Mono','Fira Code',monospace;font-size:11px}
.hdr{padding:7px 14px;display:flex;align-items:center;gap:10px;border-bottom:1px solid #161620;flex-wrap:wrap}
.hdr h1{font-size:13px;letter-spacing:2px;color:#fff}
.tag{font-size:10px;padding:2px 8px;border-radius:3px;font-weight:700}
.dot{width:7px;height:7px;border-radius:50%;display:inline-block}
.on{background:#22c55e;box-shadow:0 0 6px #22c55e}.off{background:#ef4444}
.bar{display:flex;gap:12px;padding:4px 14px;border-bottom:1px solid #0e0e16;flex-wrap:wrap;font-size:10px}
.bar label{color:#444;margin-right:2px}.bar span{color:#ddd;font-weight:600}
.btns{display:flex;gap:3px}
.btns button{padding:3px 9px;font-size:9px;font-weight:600;background:#0e0e16;color:#777;
 border:1px solid #1a1a28;border-radius:3px;cursor:pointer;font-family:inherit}
.btns button:hover{background:#16162a;color:#fff}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:1px;padding:1px;height:calc(100vh - 120px)}
.cell{background:#0a0a12;position:relative;padding:2px;overflow:hidden}
.cell canvas{width:100%!important;height:100%!important}
.ct{position:absolute;top:3px;left:6px;font-size:8px;color:#3a3a50;z-index:2;letter-spacing:.5px;text-transform:uppercase}
.tj{background:#0a0a12;border-top:1px solid #161620;padding:5px 14px;display:flex;gap:6px;align-items:center;flex-wrap:wrap}
.tj button{padding:3px 10px;font-size:9px;font-weight:600;background:#0e0e16;color:#06b6d4;
 border:1px solid #0e2030;border-radius:3px;cursor:pointer;font-family:inherit}
.tj button:hover{background:#0a1a2a;color:#0ef}
.tj label{color:#444;font-size:9px}
</style></head><body>
<div class="hdr">
 <span class="dot on" id="dot"></span>
 <h1>PUZZLEBOT</h1><span style="color:#333">LIVE</span>
 <span class="tag" id="ctag" style="color:#f97316;background:#f9731610">PID</span>
 <span id="wpi" style="color:#444;font-size:9px"></span>
 <div class="btns" style="margin-left:auto">
  <button onclick="sw('PID')">1 PID</button>
  <button onclick="sw('SMC')">2 SMC</button>
  <button onclick="sw('ISMC')">3 ISMC</button>
  <button onclick="sw('CTC')">4 CTC</button>
  <button onclick="sw('Port-Hamiltonian')">5 pH</button>
  <button onclick="sendGoal()" style="color:#22c55e">G Goal</button>
  <button onclick="sendReset()" style="color:#ef4444">R Reset</button>
 </div>
</div>
<div class="bar" id="stats">
 <div><label>t</label><span id="s0">0</span></div>
 <div><label>V</label><span id="s1">0</span></div>
 <div><label>‖e‖</label><span id="s2">0</span></div>
 <div><label>θ_e</label><span id="s3">0°</span></div>
 <div><label>v</label><span id="s4">0</span></div>
 <div><label>ω</label><span id="s5">0</span></div>
 <div><label>s_v</label><span id="s6">—</span></div>
 <div><label>s_w</label><span id="s7">—</span></div>
 <div><label>pos</label><span id="s8">(0,0)</span></div>
 <div><label>V̇</label><span id="s9">0</span></div>
 <div><label>status</label><span id="sA">—</span></div>
</div>
<div class="grid">
 <!--Row 1-->
 <div class="cell"><div class="ct">Lyapunov V(t)</div><canvas id="c0"></canvas></div>
 <div class="cell"><div class="ct">XY Trajectory — click to draw waypoints</div><canvas id="cxy"></canvas></div>
 <div class="cell"><div class="ct">Phase (e_d, ė_d) — sliding surface s_v=0</div><canvas id="c_pd"></canvas></div>
 <!--Row 2-->
 <div class="cell"><div class="ct">Distance Error ‖e‖(t)</div><canvas id="c1"></canvas></div>
 <div class="cell"><div class="ct">Control v(t), ω(t)</div><canvas id="c2"></canvas></div>
 <div class="cell"><div class="ct">Phase (θ_e, θ̇_e) — sliding surface s_w=0</div><canvas id="c_pa"></canvas></div>
 <!--Row 3-->
 <div class="cell"><div class="ct">V̇(t) — Lyapunov derivative (≤0 = stable)</div><canvas id="c3"></canvas></div>
 <div class="cell"><div class="ct">Sliding Surfaces s_v(t), s_w(t)</div><canvas id="c4"></canvas></div>
 <div class="cell"><div class="ct">Phase (v, ω) — velocity space</div><canvas id="c_pv"></canvas></div>
 <!--Row 4-->
 <div class="cell"><div class="ct">Heading Error θ_e(t)</div><canvas id="c5"></canvas></div>
 <div class="cell"><div class="ct">Perturbations F_v, τ_w</div><canvas id="c6"></canvas></div>
 <div class="cell"><div class="ct">Sliding Phase (s_v, ṡ_v) — convergence to origin</div><canvas id="c_ps"></canvas></div>
</div>
<div class="tj">
 <label>TRAJECTORY:</label>
 <button onclick="traj('circle')">⊙ Circle</button>
 <button onclick="traj('figure8')">∞ Figure-8</button>
 <button onclick="traj('square')">□ Square</button>
 <button onclick="traj('zigzag')">⩘ Zigzag</button>
 <button onclick="sendCustom()" style="color:#f97316">▶ Send Drawn</button>
 <button onclick="drawnPts=[];updateDrawn()" style="color:#666">✕ Clear</button>
 <label style="margin-left:8px;color:#555">Click XY chart to place waypoints → Send Drawn</label>
</div>
<script>
const CC={PID:'#f97316',SMC:'#06b6d4',ISMC:'#8b5cf6',CTC:'#10b981','Port-Hamiltonian':'#ef4444'};
let ctrl='PID',col='#f97316',drawnPts=[];
function sw(n){fetch('/api/switch?ctrl='+n)}
function sendGoal(){fetch('/api/goal?x=2&y=1.5')}
function sendReset(){fetch('/api/reset');drawnPts=[]}
function traj(t){fetch('/api/trajectory?type='+t);drawnPts=[]}
function sendCustom(){
 if(drawnPts.length<2)return alert('Click XY chart to place ≥2 waypoints first');
 fetch('/api/trajectory?type=custom&pts='+encodeURIComponent(JSON.stringify(drawnPts)));
}
function updateDrawn(){cxy.data.datasets[3].data=drawnPts.map(p=>({x:p[0],y:p[1]}));cxy.update();}

// Chart factories
const tO=yl=>({responsive:true,maintainAspectRatio:false,animation:{duration:0},
 scales:{x:{ticks:{color:'#2a2a40',maxTicksLimit:6,font:{size:8}},grid:{color:'#12121e'}},
  y:{ticks:{color:'#2a2a40',font:{size:8}},grid:{color:'#12121e'},
     title:{display:!!yl,text:yl||'',color:'#3a3a50',font:{size:9}}}},
 plugins:{legend:{display:false}},elements:{point:{radius:0},line:{borderWidth:1.3}}});

const phO=(xl,yl)=>({responsive:true,maintainAspectRatio:false,animation:{duration:0},
 scales:{x:{type:'linear',title:{display:true,text:xl,color:'#3a3a50',font:{size:9}},
   ticks:{color:'#2a2a40',font:{size:8}},grid:{color:'#12121e'}},
  y:{title:{display:true,text:yl,color:'#3a3a50',font:{size:9}},
   ticks:{color:'#2a2a40',font:{size:8}},grid:{color:'#12121e'}}},
 plugins:{legend:{display:false},annotation:{}},
 elements:{point:{radius:0},line:{borderWidth:1.3}}});

const xyO={responsive:true,maintainAspectRatio:false,animation:{duration:0},
 scales:{x:{type:'linear',title:{display:true,text:'x [m]',color:'#3a3a50',font:{size:9}},
   ticks:{color:'#2a2a40',font:{size:8}},grid:{color:'#12121e'}},
  y:{title:{display:true,text:'y [m]',color:'#3a3a50',font:{size:9}},
   ticks:{color:'#2a2a40',font:{size:8}},grid:{color:'#12121e'}}},
 plugins:{legend:{display:false}},elements:{point:{radius:0},line:{borderWidth:2}},
 onClick(e){
   const rx=this.scales.x.getValueForPixel(e.x),ry=this.scales.y.getValueForPixel(e.y);
   drawnPts.push([Math.round(rx*100)/100,Math.round(ry*100)/100]);
   updateDrawn();
 }};

function mk(id,opts,ds){return new Chart(document.getElementById(id),{type:'line',data:{labels:[],datasets:ds},options:opts})}

const c0=mk('c0',tO('V'),[{data:[],borderColor:col}]);
const c1=mk('c1',tO('m'),[{data:[],borderColor:col}]);
const c2=mk('c2',tO('cmd'),[{data:[],borderColor:'#22c55e',label:'v'},{data:[],borderColor:'#f97316',borderDash:[4,2],label:'ω'}]);
const c3=mk('c3',tO('dV/dt'),[{data:[],borderColor:col}]);
const c4=mk('c4',tO('s'),[{data:[],borderColor:'#06b6d4',label:'s_v'},{data:[],borderColor:'#8b5cf6',borderDash:[3,2],label:'s_w'}]);
const c5=mk('c5',tO('°'),[{data:[],borderColor:col}]);
const c6=mk('c6',tO('F/τ'),[{data:[],borderColor:'#f97316',borderWidth:1},{data:[],borderColor:'#06b6d4',borderWidth:1}]);

// Phase portraits — with sliding surface overlays
const c_pd=mk('c_pd',phO('e_d [m]','ė_d [m/s]'),[
 {data:[],borderColor:col,showLine:true},
 // Sliding surface s_v=0 line (vertical at e_d=0)
 {data:[{x:0,y:-2},{x:0,y:2}],borderColor:'#ffffff30',borderWidth:1,borderDash:[4,4],showLine:true,pointRadius:0}
]);
const c_pa=mk('c_pa',phO('θ_e [°]','θ̇_e [°/s]'),[
 {data:[],borderColor:col,showLine:true},
 // Sliding surface s_w=0 line (vertical at θ_e=0)
 {data:[{x:0,y:-500},{x:0,y:500}],borderColor:'#ffffff30',borderWidth:1,borderDash:[4,4],showLine:true,pointRadius:0}
]);
const c_pv=mk('c_pv',phO('v [m/s]','ω [rad/s]'),[
 {data:[],borderColor:col,showLine:true},
 // Origin crosshair
 {data:[{x:-1,y:0},{x:1,y:0}],borderColor:'#ffffff15',borderWidth:1,showLine:true,pointRadius:0},
 {data:[{x:0,y:-5},{x:0,y:5}],borderColor:'#ffffff15',borderWidth:1,showLine:true,pointRadius:0}
]);
const c_ps=mk('c_ps',phO('s_v','ṡ_v'),[
 {data:[],borderColor:'#06b6d4',showLine:true},
 // s=0 line
 {data:[{x:0,y:-2},{x:0,y:2}],borderColor:'#ffffff30',borderWidth:1,borderDash:[4,4],showLine:true,pointRadius:0}
]);

const cxy=mk('cxy',xyO,[
 {data:[],borderColor:col,showLine:true,borderWidth:2},
 {data:[{x:2,y:1.5}],pointRadius:8,pointStyle:'star',showLine:false,pointBackgroundColor:'#ef4444',borderColor:'#ef4444'},
 {data:[],borderColor:'#333',borderDash:[3,3],showLine:true,borderWidth:1,pointRadius:2,pointBackgroundColor:'#555'},
 {data:[],borderColor:'#f97316',borderDash:[2,2],showLine:true,borderWidth:1,pointRadius:4,pointBackgroundColor:'#f97316'}
]);

function recolor(c){
 col=CC[c]||'#fff';
 [c0,c1,c3,c5].forEach(ch=>{ch.data.datasets[0].borderColor=col});
 [c_pd,c_pa,c_pv].forEach(ch=>{ch.data.datasets[0].borderColor=col});
 cxy.data.datasets[0].borderColor=col;
 const tg=document.getElementById('ctag');
 tg.textContent=c;tg.style.color=col;tg.style.background=col+'10';
}

// ── Smooth rendering: decouple SSE data from paint via requestAnimationFrame ──
// SSE just stores the latest frame; rAF loop renders at most ~15fps.
let _pending=null, _rafId=null, _lastRender=0;

const es=new EventSource('/stream');
es.onmessage=function(ev){
 _pending=JSON.parse(ev.data);
 if(!_rafId) _rafId=requestAnimationFrame(render);
};
es.onerror=()=>{document.getElementById('dot').className='dot off'};

function render(ts){
 _rafId=null;
 // Cap render rate to ~15 fps (66 ms between frames)
 if(ts-_lastRender < 66){ _rafId=requestAnimationFrame(render); return; }
 _lastRender=ts;
 const d=_pending; if(!d) return;
 _pending=null;

 const N=d.t.length; if(N<3)return;
 if(d.ctrl!==ctrl){ctrl=d.ctrl;recolor(ctrl);}
 const lb=d.t.map(v=>v.toFixed(1));

 // Time series — use update('none') to skip animation scheduler overhead
 c0.data.labels=lb; c0.data.datasets[0].data=d.V; c0.update('none');
 c1.data.labels=lb; c1.data.datasets[0].data=d.dist; c1.update('none');
 c2.data.labels=lb; c2.data.datasets[0].data=d.v; c2.data.datasets[1].data=d.w; c2.update('none');
 c4.data.labels=lb; c4.data.datasets[0].data=d.s_v; c4.data.datasets[1].data=d.s_w; c4.update('none');
 c5.data.labels=lb; c5.data.datasets[0].data=d.angle_err.map(a=>a*180/Math.PI); c5.update('none');
 c6.data.labels=lb; c6.data.datasets[0].data=d.pv; c6.data.datasets[1].data=d.pw; c6.update('none');

 // V̇ numerical derivative
 let vdot=[];
 for(let i=1;i<N;i++){let dt=d.t[i]-d.t[i-1];if(dt<1e-6)dt=0.01;vdot.push((d.V[i]-d.V[i-1])/dt);}
 c3.data.labels=lb.slice(1); c3.data.datasets[0].data=vdot; c3.update('none');

 // Phase portraits
 let pd=[],pa=[],pv_d=[];
 for(let i=0;i<N;i++){
   pd.push({x:d.dist[i],y:d.dist_dot[i]});
   pa.push({x:d.angle_err[i]*180/Math.PI,y:d.angle_dot[i]*180/Math.PI});
   pv_d.push({x:d.v[i],y:d.w[i]});
 }
 c_pd.data.datasets[0].data=pd;
 let maxdd=1;for(let i=0;i<N;i++)maxdd=Math.max(maxdd,Math.abs(d.dist_dot[i]));
 c_pd.data.datasets[1].data=[{x:0,y:-maxdd*1.2},{x:0,y:maxdd*1.2}];
 c_pd.update('none');

 c_pa.data.datasets[0].data=pa;
 let maxad=100;for(let i=0;i<N;i++)maxad=Math.max(maxad,Math.abs(d.angle_dot[i]*180/Math.PI));
 c_pa.data.datasets[1].data=[{x:0,y:-maxad*1.2},{x:0,y:maxad*1.2}];
 c_pa.update('none');

 c_pv.data.datasets[0].data=pv_d; c_pv.update('none');

 // Sliding phase portrait: (s_v, ṡ_v)
 let ps=[];
 for(let i=1;i<N;i++){
   let dt=d.t[i]-d.t[i-1];if(dt<1e-6)dt=0.01;
   ps.push({x:d.s_v[i],y:(d.s_v[i]-d.s_v[i-1])/dt});
 }
 c_ps.data.datasets[0].data=ps;
 let maxsv=0.1;for(let p of ps)maxsv=Math.max(maxsv,Math.abs(p.y));
 c_ps.data.datasets[1].data=[{x:0,y:-maxsv*1.2},{x:0,y:maxsv*1.2}];
 c_ps.update('none');

 // XY trajectory
 let xy=[];for(let i=0;i<d.x.length;i++)xy.push({x:d.x[i],y:d.y[i]});
 cxy.data.datasets[0].data=xy;
 cxy.data.datasets[1].data=[{x:d.gx,y:d.gy}];
 if(d.waypoints&&d.waypoints.length>0)
   cxy.data.datasets[2].data=d.waypoints.map(p=>({x:p[0],y:p[1]}));
 cxy.data.datasets[3].data=drawnPts.map(p=>({x:p[0],y:p[1]}));
 cxy.update('none');

 // Stats bar
 const L=N-1;
 document.getElementById('s0').textContent=d.t[L].toFixed(1)+'s';
 document.getElementById('s1').textContent=d.V[L].toFixed(4);
 document.getElementById('s2').textContent=d.dist[L].toFixed(4)+'m';
 document.getElementById('s3').textContent=(d.angle_err[L]*180/Math.PI).toFixed(1)+'°';
 document.getElementById('s4').textContent=d.v[L].toFixed(3);
 document.getElementById('s5').textContent=d.w[L].toFixed(3);
 document.getElementById('s6').textContent=d.s_v[L].toFixed(3);
 document.getElementById('s7').textContent=d.s_w[L].toFixed(3);
 if(d.x.length>0)document.getElementById('s8').textContent='('+d.x[d.x.length-1].toFixed(2)+','+d.y[d.y.length-1].toFixed(2)+')';
 if(vdot.length>0)document.getElementById('s9').textContent=vdot[vdot.length-1].toFixed(3);
 document.getElementById('sA').textContent=d.arrived?'ARRIVED':'tracking';
 document.getElementById('dot').className='dot on';
 document.getElementById('wpi').textContent=d.wp_total>0?'WP '+d.wp+'/'+d.wp_total:'';

 // If new data arrived while we were rendering, schedule another frame
 if(_pending) _rafId=requestAnimationFrame(render);
}
</script></body></html>"""

def main(args=None):
    rclpy.init(args=args); node=DashboardNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node._srv.shutdown()
        try: node.destroy_node()
        except: pass
        try: rclpy.shutdown()
        except: pass

if __name__=='__main__': main()
