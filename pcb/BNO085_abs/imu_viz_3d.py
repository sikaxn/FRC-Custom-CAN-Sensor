import sys
import threading
import time
from collections import deque
import math

import pygame
import serial
import serial.tools.list_ports

# ---------------------------
# Config
# ---------------------------
WIN_W, WIN_H = 1280, 800
FPS = 60
PLOT_LEN = 1200
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1
BG_COLOR = (18, 18, 20)
GRID_COLOR = (40, 40, 45)
TEXT_COLOR = (230, 230, 235)
GYRO_COLORS = [(240, 120, 120), (120, 240, 120), (120, 140, 240)]

# Layout rects (menu at top)
LEFT_PAD = 16
TOP_PAD = 16
UI_RECT = pygame.Rect(LEFT_PAD, TOP_PAD, WIN_W - 2 * LEFT_PAD, 64)
PLOT_RECT = pygame.Rect(LEFT_PAD, UI_RECT.bottom + 16, 900, WIN_H - UI_RECT.bottom - 32)
SIDE_RECT = pygame.Rect(PLOT_RECT.right + 12, PLOT_RECT.y, WIN_W - PLOT_RECT.right - 28, PLOT_RECT.height)

# 3D viewport
VIEW3D_SIZE = min(380, PLOT_RECT.height // 2)
VIEW3D_RECT = pygame.Rect(PLOT_RECT.x, PLOT_RECT.y, VIEW3D_SIZE, VIEW3D_SIZE)

# ---------------------------
# 3D math helpers
# ---------------------------
def clamp(v, lo, hi): return max(lo, min(hi, v))

def euler_to_rot(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp  , cp*sr           , cp*cr           ],
    ]

def rot_vec(R, v):
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )

def project_point(v, vp_rect, fov=600.0, z_offset=4.0):
    x, y, z = v
    zc = z + z_offset
    if zc <= 0.1: zc = 0.1
    sx = vp_rect.x + vp_rect.w/2 + (fov * x) / zc
    sy = vp_rect.y + vp_rect.h/2 - (fov * y) / zc
    return (int(sx), int(sy))

# ---------------------------
# Serial reader
# ---------------------------
class SerialReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.ser = None
        self.running = False
        self.lock = threading.Lock()
        self.columns = []
        self.latest = {}
        self.buffers = {}
        self.connected_port = None

    def connect(self, port):
        self.disconnect()
        try:
            self.ser = serial.Serial(port=port, baudrate=SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
            self.running = True
            self.connected_port = port
            return True, ""
        except Exception as e:
            self.ser = None
            self.running = False
            self.connected_port = None
            return False, str(e)

    def disconnect(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected_port = None

    def clear_buffers(self):
        with self.lock:
            for k in self.buffers:
                self.buffers[k].clear()

    def ensure_buffer(self, key):
        if key not in self.buffers:
            self.buffers[key] = deque(maxlen=PLOT_LEN)

    def run(self):
        buf = b""
        while True:
            if not self.running or self.ser is None:
                time.sleep(0.05)
                continue
            try:
                chunk = self.ser.read(1024)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip().decode(errors="ignore")
                    if not line: continue
                    self._process_line(line)
            except Exception:
                self.disconnect()
                time.sleep(0.25)

    def _process_line(self, line):
        parts = [p for p in line.replace(",", " ").replace("\t", " ").split(" ") if p != ""]
        if not parts: return
        is_header = any(not self._is_float(tok) for tok in parts)
        with self.lock:
            if is_header:
                self.columns = parts
                for name in self.columns:
                    self.ensure_buffer(name)
            else:
                keys = self.columns or ["ax","ay","az","gx","gy","gz","temp"]
                for i, val in enumerate(parts[:len(keys)]):
                    if self._is_float(val):
                        f = float(val)
                        self.latest[keys[i]] = f
                        self.ensure_buffer(keys[i])
                        self.buffers[keys[i]].append(f)

    @staticmethod
    def _is_float(s):
        try: float(s); return True
        except: return False

# ---------------------------
# UI widgets
# ---------------------------
class Button:
    def __init__(self, rect, label):
        self.rect = pygame.Rect(rect); self.label = label; self.hover = False
    def draw(self, surf, font):
        base, hover, border = (60,62,70), (80,82,90), (110,110,120)
        pygame.draw.rect(surf, hover if self.hover else base, self.rect, border_radius=8)
        pygame.draw.rect(surf, border, self.rect, width=1, border_radius=8)
        txt = font.render(self.label, True, TEXT_COLOR); surf.blit(txt, txt.get_rect(center=self.rect.center))
    def handle(self, event):
        if event.type == pygame.MOUSEMOTION: self.hover = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button==1 and self.rect.collidepoint(event.pos): return True
        return False

class Dropdown:
    def __init__(self, rect, items=None, placeholder="Select COM"):
        self.rect = pygame.Rect(rect); self.items = items or []; self.placeholder = placeholder
        self.open=False; self.selected_index=None; self.hover=False; self.item_height=self.rect.height
    def set_items(self, items):
        self.items = items
        if self.selected_index is not None and self.selected_index >= len(items): self.selected_index=None
    def selected(self): return self.items[self.selected_index] if self.selected_index is not None else None
    def draw(self, surf, font):
        base, hover, border = (60,62,70), (80,82,90), (110,110,120)
        pygame.draw.rect(surf, hover if self.hover else base, self.rect, border_radius=8)
        pygame.draw.rect(surf, border, self.rect, width=1, border_radius=8)
        label = self.selected() or self.placeholder
        txt = font.render(label, True, TEXT_COLOR); surf.blit(txt,(self.rect.x+10,self.rect.y+(self.rect.height-txt.get_height())//2))
        pygame.draw.polygon(surf,TEXT_COLOR,[(self.rect.right-18,self.rect.y+self.rect.height//2-3),
                                             (self.rect.right-8,self.rect.y+self.rect.height//2-3),
                                             (self.rect.right-13,self.rect.y+self.rect.height//2+5)])
        if self.open:
            list_rect=pygame.Rect(self.rect.x,self.rect.bottom+2,self.rect.width,self.item_height*min(8,len(self.items)))
            pygame.draw.rect(surf,base,list_rect,border_radius=6); pygame.draw.rect(surf,border,list_rect,width=1,border_radius=6)
            for i,item in enumerate(self.items[:8]):
                r=pygame.Rect(list_rect.x,list_rect.y+i*self.item_height,list_rect.width,self.item_height)
                pygame.draw.rect(surf,hover if r.collidepoint(pygame.mouse.get_pos()) else base,r)
                t=font.render(item,True,TEXT_COLOR); surf.blit(t,(r.x+8,r.y+(r.height-t.get_height())//2))
    def handle(self,event):
        if event.type==pygame.MOUSEMOTION: self.hover=self.rect.collidepoint(event.pos)
        if event.type==pygame.MOUSEBUTTONDOWN and event.button==1:
            if self.rect.collidepoint(event.pos): self.open=not self.open; return("toggle",None)
            if self.open:
                y=event.pos[1]-(self.rect.bottom+2)
                if 0<=y<self.item_height*min(8,len(self.items)):
                    idx=y//self.item_height
                    if idx<len(self.items[:8]): self.selected_index=int(idx); self.open=False; return("select",self.selected())
                self.open=False
        return(None,None)

# ---------------------------
# Helpers
# ---------------------------
def list_com_ports():
    return [p.device for p in serial.tools.list_ports.comports()] or ["COM1","COM2"]

def draw_grid(surf, rect, x_div=10, y_div=8):
    pygame.draw.rect(surf, GRID_COLOR, rect, 1)
    for i in range(1,x_div): pygame.draw.line(surf,GRID_COLOR,(rect.x+int(rect.w*i/x_div),rect.y),(rect.x+int(rect.w*i/x_div),rect.bottom))
    for j in range(1,y_div): pygame.draw.line(surf,GRID_COLOR,(rect.x,rect.y+int(rect.h*j/y_div)),(rect.right,rect.y+int(rect.h*j/y_div)))

def plot_series(surf, rect, data, color, vmin, vmax):
    if not data or len(data)<2: return
    step=max(1,rect.w/max(1,PLOT_LEN-1)); points=[]
    for i,v in enumerate(list(data)[-PLOT_LEN:]):
        x=rect.x+i*step; vv=max(vmin,min(v,vmax)); t=(vv-vmin)/(vmax-vmin) if vmax>vmin else 0.5
        y=rect.bottom-t*rect.h; points.append((x,y))
    if len(points)>=2: pygame.draw.lines(surf,color,False,points,2)

def text_line(surf,font,x,y,label,value,color=TEXT_COLOR):
    txt=font.render(f"{label}: {value}",True,color); surf.blit(txt,(x,y)); return y+txt.get_height()+6

# ---------------------------
# Main
# ---------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("IMU Visualizer")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas",18)
    big = pygame.font.SysFont("consolas",22,bold=True)

    reader = SerialReader(); reader.start()

    # UI
    dd_ports = Dropdown((UI_RECT.x, UI_RECT.y, 280, 40), list_com_ports())
    btn_refresh=Button((dd_ports.rect.right+10,UI_RECT.y,100,40),"Refresh")
    btn_connect=Button((btn_refresh.rect.right+10,UI_RECT.y,120,40),"Connect")
    btn_disconnect=Button((btn_connect.rect.right+10,UI_RECT.y,140,40),"Disconnect")
    btn_clear=Button((btn_disconnect.rect.right+10,UI_RECT.y,100,40),"Clear")

    status_msg="Idle"; status_color=(180,180,180)

    # Cube model
    s=0.8
    cube_vertices=[(-s,-s,-s),( s,-s,-s),( s, s,-s),(-s, s,-s),(-s,-s, s),( s,-s, s),( s, s, s),(-s, s, s)]
    cube_edges=[(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    roll=pitch=yaw=0.0; alpha=0.98; last_time=time.perf_counter()

    running=True
    while running:
        for event in pygame.event.get():
            if event.type==pygame.QUIT: running=False
            if btn_refresh.handle(event): dd_ports.set_items(list_com_ports())
            if btn_connect.handle(event):
                port=dd_ports.selected()
                if port:
                    ok,err=reader.connect(port)
                    if ok: status_msg=f"Connected {port}"; status_color=(120,255,120)
                    else: status_msg=f"Connect failed: {err}"; status_color=(255,120,120)
            if btn_disconnect.handle(event): reader.disconnect(); status_msg="Disconnected"; status_color=(200,200,200)
            if btn_clear.handle(event): reader.clear_buffers(); status_msg="Cleared"; status_color=(180,200,255)
            _a,_b=dd_ports.handle(event)

        screen.fill(BG_COLOR)

        # Latest
        with reader.lock:
            gx=reader.latest.get("gx",0.0); gy=reader.latest.get("gy",0.0); gz=reader.latest.get("gz",0.0)
            ax=reader.latest.get("ax",0.0); ay=reader.latest.get("ay",0.0); az=reader.latest.get("az",1.0)
            latest=dict(reader.latest)

        # Orientation update
        now=time.perf_counter(); dt=clamp(now-last_time,0.0005,0.05); last_time=now
        gr, gp, gzrad = math.radians(gx), math.radians(gy), math.radians(gz)
        roll_gyro=roll+gr*dt; pitch_gyro=pitch+gp*dt; yaw+=gzrad*dt
        acc_roll=math.atan2(ay,az); acc_pitch=math.atan2(-ax,math.sqrt(ay*ay+az*az))
        roll=alpha*roll_gyro+(1-alpha)*acc_roll; pitch=alpha*pitch_gyro+(1-alpha)*acc_pitch
        R=euler_to_rot(roll,pitch,yaw)

        # Draw cube
        pygame.draw.rect(screen,GRID_COLOR,VIEW3D_RECT,1)
        cx,cy=VIEW3D_RECT.center; pygame.draw.line(screen,GRID_COLOR,(VIEW3D_RECT.left,cy),(VIEW3D_RECT.right,cy))
        pygame.draw.line(screen,GRID_COLOR,(cx,VIEW3D_RECT.top),(cx,VIEW3D_RECT.bottom))
        pts2d=[project_point(rot_vec(R,v),VIEW3D_RECT,fov=500,z_offset=3.0) for v in cube_vertices]
        for i,j in cube_edges: pygame.draw.line(screen,(180,220,255),pts2d[i],pts2d[j],2)
        label3d=big.render("IMU Orientation",True,TEXT_COLOR); screen.blit(label3d,(VIEW3D_RECT.x,VIEW3D_RECT.y-26))

        # Draw gyro plot
        draw_grid(screen,PLOT_RECT)
        with reader.lock:
            for idx,key in enumerate(["gx","gy","gz"]):
                if key in reader.buffers:
                    plot_series(screen,PLOT_RECT,reader.buffers[key],GYRO_COLORS[idx],-500,500)

        # Side info
        sx,sy=SIDE_RECT.x+12,SIDE_RECT.y+12
        sy=text_line(screen,big,sx,sy,"Latest","")
        def g(k,f="{:.2f}"): return f.format(latest[k]) if k in latest else "—"
        sy=text_line(screen,font,sx,sy,"ax",g("ax")); sy=text_line(screen,font,sx,sy,"ay",g("ay")); sy=text_line(screen,font,sx,sy,"az",g("az"))
        sy=text_line(screen,font,sx,sy,"gx",g("gx")); sy=text_line(screen,font,sx,sy,"gy",g("gy")); sy=text_line(screen,font,sx,sy,"gz",g("gz"))
        sy=text_line(screen,font,sx,sy,"temp",g("temp","{:.1f}"))

        # UI bar
        dd_ports.draw(screen,font); btn_refresh.draw(screen,font); btn_connect.draw(screen,font); btn_disconnect.draw(screen,font); btn_clear.draw(screen,font)
        status=font.render(status_msg,True,status_color); screen.blit(status,(UI_RECT.x,UI_RECT.bottom+6))

        pygame.display.flip(); clock.tick(FPS)

    reader.disconnect(); pygame.quit(); sys.exit()

if __name__=="__main__":
    main()
