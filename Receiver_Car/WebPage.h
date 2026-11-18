/*
  ============================================================================
   WebPage.h  -  control dashboard served by Receiver_Car.ino
  ============================================================================
   Kept in a separate header on purpose: the Arduino .ino preprocessor scans
   the sketch for function signatures before compiling, and the JavaScript in
   this page ("async function poll()") makes it inject invalid prototypes.
   Keeping the markup out of the .ino keeps both the Arduino IDE and
   PlatformIO builds clean.

   PROGMEM: the page stays in flash and is never copied into RAM.
  ============================================================================
*/
#pragma once
#include <Arduino.h>

static const char PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#0b1020">
<title>Smart Car</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;background:radial-gradient(120% 80% at 50% 0%,#15224a 0%,#080b16 60%,#05070d 100%);
color:#eaf0ff;min-height:100vh;display:flex;align-items:center;justify-content:center;padding:18px;overflow-x:hidden}
.wrap{width:100%;max-width:430px}
.top{display:flex;align-items:center;justify-content:space-between;margin-bottom:6px}
h1{font-size:20px;font-weight:700;letter-spacing:.3px}
.dot{width:9px;height:9px;border-radius:50%;background:#ff4d4d;box-shadow:0 0 10px #ff4d4d;margin-right:7px;display:inline-block;transition:.3s}
.dot.on{background:#33e08a;box-shadow:0 0 10px #33e08a}
.pill{display:flex;align-items:center;font-size:11px;letter-spacing:.6px;text-transform:uppercase;background:#ffffff10;border:1px solid #ffffff1f;padding:6px 11px;border-radius:99px}
.badge{margin:12px 0 16px;padding:11px 14px;border-radius:14px;font-size:13px;line-height:1.45;background:#ffffff0d;border:1px solid #ffffff1a}
.badge.warn{background:#ff9f1c14;border-color:#ff9f1c55;color:#ffcf8a}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:16px}
.stat{background:#ffffff0a;border:1px solid #ffffff14;border-radius:14px;padding:10px 12px}
.stat b{display:block;font-size:10px;letter-spacing:1px;color:#8fa3c8;font-weight:600;text-transform:uppercase}
.stat span{font-size:18px;font-weight:700}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;aspect-ratio:1;max-width:330px;margin:0 auto}
.btn{border:1px solid #ffffff24;background:linear-gradient(180deg,#ffffff1a,#ffffff08);color:#eaf0ff;border-radius:20px;
display:flex;align-items:center;justify-content:center;font-size:30px;font-weight:600;cursor:pointer;user-select:none;
touch-action:none;transition:transform .07s,background .12s,box-shadow .12s;box-shadow:0 6px 18px #0008}
.btn:active,.btn.held{transform:scale(.94);background:linear-gradient(180deg,#4da3ff66,#4da3ff22);box-shadow:0 0 22px #4da3ff55}
.btn.stop{background:linear-gradient(180deg,#ff5f5726,#ff5f570a);border-color:#ff5f5755;color:#ff8f88;font-size:26px}
.btn.stop:active,.btn.stop.held{background:#ff5f5744;box-shadow:0 0 22px #ff5f5766}
.spacer{visibility:hidden}
.foot{text-align:center;margin-top:18px;font-size:11px;color:#7f8fb0;line-height:1.7}
kbd{background:#ffffff14;border:1px solid #ffffff22;border-radius:5px;padding:1px 5px;font-size:10px}
.btn.off{opacity:.35;pointer-events:none}
.warn{margin:0 0 14px;padding:9px 12px;border-radius:12px;font-size:12px;
  background:#ff9f1c1f;border:1px solid #ff9f1c66;color:#ffcf8a;text-align:center;display:none}
.warn.on{display:block}
</style></head><body>
<div class="wrap">
  <div class="top">
    <h1>Smart Car</h1>
    <div class="pill"><i class="dot" id="dot"></i><span id="link">LINK</span></div>
  </div>
  <div class="badge" id="badge">Connecting&hellip;</div>
  <div class="stats">
    <div class="stat"><b>Mode</b><span id="mode">&mdash;</span></div>
    <div class="stat"><b>Distance</b><span id="dist">&mdash;</span></div>
  </div>
  <div class="warn" id="warn" style="display:none">Obstacle ahead - forward blocked (left / right / back still work)</div>
  <div class="pad" id="pad">
    <div class="spacer"></div>
    <div class="btn" data-dir="forward">&#9650;</div>
    <div class="spacer"></div>
    <div class="btn" data-dir="left">&#9664;</div>
    <div class="btn stop" data-dir="stop">&#9632;</div>
    <div class="btn" data-dir="right">&#9654;</div>
    <div class="spacer"></div>
    <div class="btn" data-dir="backward">&#9660;</div>
    <div class="spacer"></div>
  </div>
  <div class="foot">Hold a button to drive &middot; release to stop<br>
  Keyboard: <kbd>&#8593;</kbd> <kbd>&#8595;</kbd> <kbd>&#8592;</kbd> <kbd>&#8594;</kbd> <kbd>space</kbd></div>
</div>
<script>
const dot=document.getElementById('dot'),link=document.getElementById('link'),
      badge=document.getElementById('badge'),modeEl=document.getElementById('mode'),
      distEl=document.getElementById('dist'),pad=document.getElementById('pad');
let held=null,hb=null,inWeb=false;

function send(path){
  fetch(path,{cache:'no-store'}).catch(()=>{});
}
function start(dir){
  if(!inWeb) return;
  held=dir;
  send('/'+dir);
  clearInterval(hb);
  if(dir!=='stop') hb=setInterval(()=>{ if(held) send('/'+held); },150);
}
function stop(){
  clearInterval(hb); hb=null;
  if(held&&held!=='stop') send('/stop');
  held=null;
}
pad.querySelectorAll('.btn').forEach(b=>{
  b.addEventListener('pointerdown',e=>{
    e.preventDefault();
    b.setPointerCapture(e.pointerId);
    b.classList.add('held');
    start(b.dataset.dir);
  });
  const up=e=>{e.preventDefault();b.classList.remove('held');stop();};
  b.addEventListener('pointerup',up);
  b.addEventListener('pointercancel',up);
  b.addEventListener('lostpointercapture',up);
});
window.addEventListener('blur',stop);
document.addEventListener('visibilitychange',()=>{ if(document.hidden) stop(); });

const keyMap={ArrowUp:'forward',ArrowDown:'backward',ArrowLeft:'left',ArrowRight:'right',' ':'stop'};
document.addEventListener('keydown',e=>{
  const d=keyMap[e.key]; if(!d) return; e.preventDefault(); if(!e.repeat) start(d);
});
document.addEventListener('keyup',e=>{
  const d=keyMap[e.key]; if(!d) return; e.preventDefault(); stop();
});

async function poll(){
  try{
    const r=await fetch('/status',{cache:'no-store'});
    const j=await r.json();
    inWeb=(j.mode===1);
    dot.classList.toggle('on',inWeb);
    link.textContent=inWeb?'LIVE':'IDLE';
    modeEl.textContent=j.modeName;
    distEl.textContent=(j.dist>=0)?j.dist+' cm':'--';
    const warn=document.getElementById('warn');
    const fwdBtn=document.querySelector('.btn[data-dir="forward"]');
    if(j.obstacle){ warn.classList.add('on'); if(fwdBtn) fwdBtn.classList.add('off'); }
    else { warn.classList.remove('on'); if(fwdBtn) fwdBtn.classList.remove('off'); }
    if(inWeb){
      badge.className='badge';
      badge.innerHTML='Connected &bull; hold a button to drive';
      pad.classList.remove('off');
    }else{
      badge.className='badge warn';
      badge.innerHTML='Car is in <b>'+j.modeName+'</b> mode.<br>Press the glove button until the OLED shows <b>MOBILE WEB</b>.';
      pad.classList.add('off');
      stop();
    }
  }catch(err){
    dot.classList.remove('on'); link.textContent='OFF';
    badge.className='badge warn';
    badge.textContent='Car not responding. Reconnect to Smart_Car_AP.';
    pad.classList.add('off');
    inWeb=false;
  }
}
poll(); setInterval(poll,1000);
</script></body></html>
)HTML";

