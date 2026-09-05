"use strict";
const $=id=>document.getElementById(id);
let session=null,rows=[],cursor=0,playing=false,playStart=0,playIndex=0,connected=false,lastMessage=0;
const phaseNames={0:"IDLE",1:"FOLD",4:"REST"},startNames={0:"대기",1:"준비 중",2:"READY",3:"카운트다운",4:"제어 중",5:"FAULT"};
const eventNames={1:"접기 시작",2:"도착",3:"접기 시간 초과",4:"대기 종료"};
const listeners={};
const LG=window.LG={el:$,GEOM:null,on(name,fn){(listeners[name]??=[]).push(fn);},cur:()=>rows.length?cursor:-1,
 poseAt(i){const r=rows[i];return {phi:r.phi,alpha:r.ank-r.phi,theta:r.ank-r.phi+r.del_now};}};
function icons(){if(window.lucide)lucide.createIcons();}
function notice(s){$("notice").textContent=s||"";}
async function api(name,data){const r=await fetch('/api/'+name,{method:data?'POST':'GET',headers:{'Content-Type':'application/json'},body:data?JSON.stringify(data):undefined});const j=await r.json();if(!r.ok)throw Error(j.error||r.statusText);return j;}
function act(fn){return async()=>{try{notice('');await fn();}catch(e){notice(e.message);}};}
function setSession(data){session=data;rows=data.rows||[];cursor=0;playing=false;$("follow").checked=false;$("scrub").max=Math.max(0,rows.length-1);$("sessionName").textContent=data.name||'';$("metadata").textContent=JSON.stringify({kind:data.kind,config:data.config,inputs:data.inputs,hashes:data.hashes,notes:data.notes},null,2);if(data.geom){LG.GEOM=data.geom;for(const fn of listeners.hello||[])fn(data);}renderEvents();update();}
function update(){if(!rows.length)return;cursor=Math.max(0,Math.min(rows.length-1,cursor));const r=rows[cursor];$("scrub").value=cursor;$("time").textContent=(r.t_ms/1000).toFixed(3)+' s';$("sampleNo").textContent=`${cursor+1} / ${rows.length}`;$("frameState").textContent=r.start!==undefined?startNames[r.start]:phaseNames[r.phase];
 const fields=[['φ',r.phi],['발목 상대각',r.ank],['하체 α',r.ank-r.phi],['힙 δ',r.del_now],['β',r.beta],['φ̇',r.dphi],['β̇',r.dbeta],['Â',r.Ahat],['목표 유지각',r.hold],['프로파일 목표',r.ref],['Â 비교 오차',r.error_Ahat],['유지각 비교 오차',r.error_hold]];
 $('values').replaceChildren(...fields.map(([key,val])=>{const tr=document.createElement('tr'),a=document.createElement('td'),b=document.createElement('td');a.textContent=key;b.textContent=Number.isFinite(val)?(key.includes('오차')?val.toExponential(2):val.toFixed(4)):'—';tr.append(a,b);return tr;}));
 const sum=session.summary||{};$("result").textContent=sum.samples?`${sum.samples} 샘플 · ${sum.first_mismatch==null?'동일입력 일치':'불일치 '+sum.first_mismatch} · ${sum.fallen?'낙하':'시험 종료까지 유지'}`:(r.replay_valid===0?'재계산 비교 불완전':'기록 재생');draw();}
function renderEvents(){const items=session.events||[];$('events').replaceChildren(...items.slice(-600).map(e=>{const b=document.createElement('button');const label=eventNames[e.event]||e.event;b.textContent=`${Number(e.t_ms||0).toFixed(0)} ms  ${label}`;b.onclick=()=>jump(e.seq);return b;}));}
function jump(i){playing=false;$('follow').checked=false;cursor=Number(i)||0;update();}
function canvas(id){const c=$(id),w=c.clientWidth,h=c.clientHeight,d=devicePixelRatio;if(c.width!==Math.round(w*d)||c.height!==Math.round(h*d)){c.width=Math.round(w*d);c.height=Math.round(h*d);}const g=c.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,w,h);return {c,g,w,h};}
function plane(id,pred){const {g,w,h}=canvas(id);if(!rows.length)return;const r=rows[cursor];const xkey=pred?'beta_pred':'beta',ykey=pred?'phi_pred':'phi';const hist=rows.slice(Math.max(0,cursor-250),cursor+1);let lim=3;for(const q of hist)lim=Math.max(lim,Math.abs(q[xkey]||0)*1.15,Math.abs(q[ykey]||0)*1.15);lim=Math.ceil(lim);const scale=Math.min(w-54,h-26)/(2*lim),X=x=>w/2+x*scale,Y=y=>h/2-y*scale;
 g.strokeStyle='#e3e8ec';g.lineWidth=1;for(let k=-2;k<=2;k++){let a=k*lim/2;g.beginPath();g.moveTo(X(-lim),Y(a));g.lineTo(X(lim),Y(a));g.moveTo(X(a),Y(-lim));g.lineTo(X(a),Y(lim));g.stroke();}
 g.strokeStyle='#96a4af';g.beginPath();g.moveTo(12,Y(0));g.lineTo(w-12,Y(0));g.moveTo(X(0),8);g.lineTo(X(0),h-8);g.stroke();
 const slope=pred?-(1/(session.config.wphi||.608)):session.plane?.r;
 const intercept=pred?-(session.config.offset||0)/(session.config.wphi||.608):0;
 if(Number.isFinite(slope)){g.strokeStyle='#23937b';g.lineWidth=1.5;g.beginPath();g.moveTo(X(-lim),Y(-lim*slope+intercept));g.lineTo(X(lim),Y(lim*slope+intercept));g.stroke();}
 g.strokeStyle=pred?'#a64262':'#356bc5';g.lineWidth=1;g.beginPath();hist.forEach((q,i)=>{const x=X(q[xkey]||0),y=Y(q[ykey]||0);i?g.lineTo(x,y):g.moveTo(x,y);});g.stroke();g.fillStyle=pred?'#b83559':'#356bc5';g.beginPath();g.arc(X(r[xkey]||0),Y(r[ykey]||0),4,0,Math.PI*2);g.fill();g.fillStyle='#647783';g.font='10px Arial';g.fillText('+'+lim+'°',w-34,Y(0)-4);g.fillText('φ',X(0)+5,12);g.fillText('β',w-12,Y(0)+12);}
function trace(){const {g,w,h}=canvas('trace');if(!rows.length)return;const ta=rows[0].t_ms,tb=rows[rows.length-1].t_ms;const X=t=>35+(w-45)*(t-ta)/Math.max(1,tb-ta);let lim=5;for(const r of rows)lim=Math.max(lim,Math.abs(r.hold||0),Math.abs(r.del_now||0),Math.abs(r.Ahat||0));const Y=v=>h/2-v*(h-24)/(2*lim);g.strokeStyle='#d6dfe5';g.beginPath();g.moveTo(35,Y(0));g.lineTo(w,Y(0));g.stroke();const stride=Math.max(1,Math.floor(rows.length/w));for(const [key,color] of [['hold','#0b8268'],['del_now','#326cca'],['Ahat','#bc3151']]){g.strokeStyle=color;g.lineWidth=1.2;g.beginPath();for(let i=0;i<rows.length;i+=stride){const x=X(rows[i].t_ms),y=Y(rows[i][key]||0);i?g.lineTo(x,y):g.moveTo(x,y);}g.stroke();}g.strokeStyle='#202a31';g.beginPath();g.moveTo(X(rows[cursor].t_ms),0);g.lineTo(X(rows[cursor].t_ms),h);g.stroke();g.fillStyle='#63727c';g.font='11px Arial';g.fillText(lim.toFixed(1)+'°',0,15);g.fillText((-lim).toFixed(1)+'°',0,h-5);}
function draw(){plane('position',false);plane('prediction',true);trace();}
function exportSession(){if(!session)return;const blob=new Blob([JSON.stringify(session)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=(session.name||'v23')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
async function files(){const j=await api('files');$('files').replaceChildren(...j.files.map(v=>new Option(v,v)));}
async function ports(){const j=await api('ports');$('ports').replaceChildren(...j.ports.map(p=>new Option(p.device,p.device)));if(!j.ports.length)$('ports').add(new Option('USB 없음',''));}
async function boot(){
 await files();await ports();icons();
 const ws=new WebSocket('ws://'+location.host+'/ws23');
 ws.onmessage=e=>{
  lastMessage=performance.now();const m=JSON.parse(e.data);
  if(m.type==='session')setSession(m.data);
  if(m.type==='append'){
   if(!session)return;
   rows.push(...m.rows);session.events=m.events;session.config=m.config;session.plane=m.plane;session.notes=m.notes;
   $('scrub').max=rows.length-1;if($('follow').checked)cursor=rows.length-1;renderEvents();update();
  }
  if(m.type==='status'){
   connected=m.connected;$('status').textContent=m.stale?'USB 데이터 지연':m.connected?'USB 연결':m.busy?'검증 실행 중':'오프라인 검토';
   $('simulate').disabled=m.busy||m.connected;if(m.message)notice(m.message);
  }
 };
 ws.onclose=()=>{$('status').textContent='서버 연결 끊김';notice('새로고침하여 다시 연결하세요.');};
}
$('load').onclick=act(()=>api('load',{name:$('files').value}));$('refresh').onclick=act(ports);$('connect').onclick=act(()=>api('connect',{port:$('ports').value}));$('disconnect').onclick=act(()=>api('disconnect',{}));
$('upload').onclick=()=>$('file').click();$('file').onchange=act(async()=>{const f=$('file').files[0];if(f)await api('upload',{name:f.name,text:await f.text()});});$('download').onclick=exportSession;
$('simulate').onclick=act(async()=>{const p={};for(const k of ['seconds','beta','phi','gain','noise','release_ms','arm_ms'])p[k]=Number($(k).value);$('simulate').disabled=true;try{await api('simulate',p);await files();}finally{$('simulate').disabled=false;}});
document.querySelectorAll('[data-cmd]').forEach(b=>b.onclick=act(()=>api('command',{command:b.dataset.cmd})));$('send').onclick=act(()=>api('command',{command:$('command').value}));$('command').onkeydown=e=>{if(e.key==='Enter')$('send').click();};
$('prev').onclick=()=>jump(cursor-1);$('next').onclick=()=>jump(cursor+1);$('scrub').oninput=()=>jump($('scrub').value);$('play').onclick=()=>{playing=!playing;if(playing){if(cursor>=rows.length-1)cursor=0;playStart=performance.now();playIndex=cursor;}};
$('firstMismatch').onclick=()=>{const i=rows.findIndex(r=>r.mismatch);if(i>=0)jump(i);else notice('비교된 샘플에 불일치가 없습니다.');};$('side').onclick=()=>LG.cam.side(false);$('iso').onclick=()=>LG.cam.iso(false);
$('trace').onclick=e=>{if(!rows.length)return;const rect=$('trace').getBoundingClientRect(),f=Math.max(0,Math.min(1,(e.clientX-rect.left-35)/(rect.width-45)));jump(Math.round(f*(rows.length-1)));};
$('analyze').onclick=act(async()=>{const result=await api('analyze',{tool:$('analysisTool').value,args:{t0:Number($('t0').value),t1:Number($('t1').value)}});$('analysisResult').textContent=JSON.stringify(result,null,2);});
document.addEventListener('keydown',e=>{if(/INPUT|SELECT|TEXTAREA/.test(e.target.tagName))return;if(e.key==='ArrowLeft'){e.preventDefault();jump(cursor-1);}if(e.key==='ArrowRight'){e.preventDefault();jump(cursor+1);}});
window.addEventListener('resize',draw);
function frame(now){if(playing&&rows.length){const t=rows[playIndex].t_ms+(now-playStart)*Number($('speed').value);while(cursor<rows.length-1&&rows[cursor+1].t_ms<=t)cursor++;if(cursor===rows.length-1)playing=false;update();}if(LG.render3d)LG.render3d();requestAnimationFrame(frame);}
requestAnimationFrame(frame);boot().catch(e=>notice(e.message));
