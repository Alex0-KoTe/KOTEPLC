#ifndef INDEX_HTML_H
#define INDEX_HTML_H

// Главная страница веб-интерфейса KOTELPC ( PROGMEM , без блокирующего чтения FS ).
// Тёмная тема, строгий дизайн , скруглённые кнопки. Три вкладки :
//   1 ) Настройки сети   2 ) Состояние входов - выходов   3 ) Файлы LittleFS
static const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KOTELPC</title>
<style>
:root{--bg:#0f1115;--panel:#171a21;--panel2:#1d212b;--line:#2a2f3d;--fg:#d7dae2;
--mut:#8b93a7;--acc:#4f8cff;--ok:#3ecf8e;--err:#ff6b6b;--warn:#ffb454}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
font:14px/1.5 "Segoe UI",system-ui,-apple-system,Roboto,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:14px 20px;
background:var(--panel);border-bottom:1px solid var(--line)}
header .logo{width:30px;height:30px;border-radius:50%;background:var(--acc);
display:grid;place-items:center;font-weight:700;color:#fff}
h1{font-size:16px;margin:0;font-weight:600;letter-spacing:.4px}
.badge{margin-left:auto;font-size:12px;color:var(--mut)}
nav{display:flex;gap:8px;padding:12px 20px 0}
.tab{padding:8px 18px;border-radius:20px;border:1px solid var(--line);
background:var(--panel);color:var(--mut);cursor:pointer;font-size:13px;transition:.15s}
.tab:hover{color:var(--fg);border-color:var(--acc)}
.tab.active{background:var(--acc);border-color:var(--acc);color:#fff}
main{padding:16px 20px;max-width:900px;margin:0 auto}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;
padding:16px 18px;margin-bottom:14px}
.card h2{margin:0 0 12px;font-size:14px;font-weight:600;color:var(--fg)}
label{display:block;font-size:12px;color:var(--mut);margin:10px 0 4px}
input[type=text],input[type=password]{width:100%;padding:9px 12px;border-radius:10px;
border:1px solid var(--line);background:var(--panel2);color:var(--fg);outline:none}
input:focus{border-color:var(--acc)}
button{padding:9px 20px;border-radius:20px;border:1px solid var(--acc);
background:var(--acc);color:#fff;cursor:pointer;font-size:13px;transition:.15s}
button:hover{filter:brightness(1.15)}
button.sec{background:transparent;color:var(--acc)}
button.danger{border-color:var(--err);background:transparent;color:var(--err)}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.status{font-size:13px;margin-top:10px;min-height:18px}
.status.ok{color:var(--ok)}.status.err{color:var(--err)}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{padding:7px 8px;text-align:left;border-bottom:1px solid var(--line)}
th{color:var(--mut);font-weight:500;font-size:12px}
.pins{display:grid;grid-template-columns:repeat(auto-fill,minmax(96px,1fr));gap:8px}
.pin{border:1px solid var(--line);border-radius:10px;padding:8px 10px;
background:var(--panel2);display:flex;justify-content:space-between;align-items:center}
.pin b{font-size:12px;color:var(--mut);font-weight:500}
.dot{width:12px;height:12px;border-radius:50%;background:#3a4052}
.dot.on{background:var(--ok);box-shadow:0 0 8px var(--ok)}
.dot.relay.on{background:var(--warn);box-shadow:0 0 8px var(--warn)}
.kv{display:grid;grid-template-columns:130px 1fr;gap:4px 12px;font-size:13px}
.kv span:nth-child(odd){color:var(--mut)}
.hidden{display:none}
#drop{border:1px dashed var(--line);border-radius:12px;padding:14px;text-align:center;
color:var(--mut);margin-bottom:10px}
</style></head><body>
<header><div class="logo">K</div><h1>KOTELPC</h1><div class="badge" id="fw">FW —</div></header>
<nav>
<div class="tab active" data-t="net">Сеть</div>
<div class="tab" data-t="io">Входы / Выходы</div>
<div class="tab" data-t="files">Файлы</div>
</nav>
<main>
<section id="t-net">
<div class="card"><h2>Настройки Wi-Fi</h2>
<label>SSID сети</label><input type="text" id="ssid" placeholder="имя точки доступа">
<label>Пароль</label><input type="password" id="pass" placeholder="оставьте пустым для открытой сети">
<div class="row" style="margin-top:14px">
<button id="saveNet">Сохранить и подключить</button>
<span class="status" id="netMsg"></span></div></div>
<div class="card"><h2>Текущее подключение</h2><div class="kv" id="wifiInfo">—</div></div>
</section>
<section id="t-io" class="hidden">
<div class="card"><h2>Дискретные входы DI0–DI7</h2><div class="pins" id="diGrid"></div></div>
<div class="card"><h2>Релейные выходы RL0–RL7</h2><div class="pins" id="doGrid"></div></div>
<div class="card"><h2>Устройство</h2><div class="kv" id="devInfo">—</div></div>
</section>
<section id="t-files" class="hidden">
<div class="card"><h2>Загрузка файлов в LittleFS (/www)</h2>
<div id="drop">перетащите файлы сюда или выберите ниже</div>
<input type="file" id="fileInput" multiple style="display:none">
<div class="row"><button class="sec" id="pickBtn">Выбрать файлы</button>
<span class="status" id="upMsg"></span></div></div>
<div class="card"><h2>Файлы на устройстве</h2>
<table><thead><tr><th>Имя</th><th>Размер</th><th></th></tr></thead>
<tbody id="fileList"></tbody></table></div>
<div class="card hidden" id="viewCard"><h2 id="viewName"></h2>
<pre id="viewBody" style="white-space:pre-wrap;max-height:400px;overflow:auto;font-size:12px"></pre></div>
</section>
</main>
<script>
let curTab='net';
document.querySelectorAll('.tab').forEach(t=>t.onclick=()=>{
 curTab=t.dataset.t;
 document.querySelectorAll('.tab').forEach(x=>x.classList.toggle('active',x===t));
 ['net','io','files'].forEach(k=>document.getElementById('t-'+k)
   .classList.toggle('hidden',k!==curTab));
 if(curTab==='files')loadFiles();});
function el(tag,cls,txt){const e=document.createElement(tag);if(cls)e.className=cls;
 if(txt!=null)e.textContent=txt;return e;}
async function api(p,opt){const r=await fetch(p,opt);if(!r.ok)throw new Error(r.status);return r.json();}
function fmtSize(n){return n>=1024?(n/1024).toFixed(1)+' KiB':n+' B';}
function pins(div,prefix,word,relay){const g=document.getElementById(div);g.innerHTML='';
 for(let i=0;i<8;i++){const c=el('div','pin');c.appendChild(el('b',null,prefix+i));
 const d=el('span','dot'+(relay?' relay':''));if(word&(1<<i))d.classList.add('on');
 c.appendChild(d);g.appendChild(c);}}
async function tick(){try{
 const s=await api('/api/status');
 document.getElementById('fw').textContent='FW '+s.fw;
 const w=s.wifi;
 document.getElementById('wifiInfo').innerHTML=
  '<span>Режим</span><span>'+(w.mode==='ap'?'Точка доступа':'Клиент (STA)')+'</span>'+
  '<span>Статус</span><span>'+(w.connected?'подключён':'нет связи')+'</span>'+
  '<span>SSID</span><span>'+(w.ssid||'—')+'</span>'+
  '<span>IP</span><span>'+w.ip+'</span>'+
  '<span>RSSI</span><span>'+(w.connected?w.rssi+' dBm':'—')+'</span>';
 if(document.getElementById('ssid').value===''&&w.ssid)document.getElementById('ssid').value=w.ssid;
 pins('diGrid','DI',s.io.di,false);pins('doGrid','RL',s.io.do,true);
 const fs_=s.fs;
 document.getElementById('devInfo').innerHTML=
  '<span>MCP23017</span><span>'+(s.io.mcp_ready?'онлайн':'офлайн')+'</span>'+
  '<span>Heap свободно</span><span>'+fmtSize(s.heap_free)+'</span>'+
  '<span>Аптайм</span><span>'+Math.floor(s.uptime_s/60)+' мин '+(s.uptime_s%60)+' с</span>'+
  '<span>LittleFS занято</span><span>'+(fs_.ok?fmtSize(fs_.used)+' из '+fmtSize(fs_.total):'не смонтирован')+'</span>';
}catch(e){document.getElementById('fw').textContent='FW — (нет связи)';}}
setInterval(tick,2000);tick();
document.getElementById('saveNet').onclick=async()=>{
 const m=document.getElementById('netMsg');m.className='status';m.textContent='Применение…';
 const q=new URLSearchParams({ssid:document.getElementById('ssid').value.trim(),
  pass:document.getElementById('pass').value});
 try{await api('/api/network?'+q.toString(),{method:'POST'});
  m.className='status ok';m.textContent='Настройки сохранены, подключаюсь…';}
 catch(e){m.className='status err';m.textContent='Ошибка: '+e.message;}};
async function loadFiles(){const tb=document.getElementById('fileList');tb.innerHTML='';
 try{const list=await api('/api/fs/list');
  if(!list.length){const tr=el('tr');tr.innerHTML='<td colspan="3" style="color:var(--mut)">пусто</td>';tb.appendChild(tr);return;}
  for(const f of list){const tr=el('tr');
   tr.appendChild(el('td',null,f.name));tr.appendChild(el('td',null,fmtSize(f.size)));
   const td=el('td');const vw=el('button','sec','Просмотр');vw.style.marginRight='8px';
   vw.onclick=async()=>{const r=await fetch('/api/fs/file?path=/www/'+encodeURIComponent(f.name));
    document.getElementById('viewCard').classList.remove('hidden');
    document.getElementById('viewName').textContent=f.name;
    document.getElementById('viewBody').textContent=(await r.text()).slice(0,60000);};
   const del=el('button','danger','Удалить');
   del.onclick=async()=>{await fetch('/api/fs/delete?path=/www/'+encodeURIComponent(f.name),{method:'POST'});loadFiles();};
   td.appendChild(vw);td.appendChild(del);tr.appendChild(td);tb.appendChild(tr);}
 }catch(e){tb.innerHTML='<td colspan="3" style="color:var(--err)">ошибка списка</td>';}}
document.getElementById('pickBtn').onclick=()=>document.getElementById('fileInput').click();
document.getElementById('fileInput').onchange=e=>uploadAll([...e.target.files]);
document.getElementById('drop').ondragover=e=>{e.preventDefault();
 document.getElementById('drop').style.borderColor='var(--acc)';};
document.getElementById('drop').ondragleave=()=>
 document.getElementById('drop').style.borderColor='var(--line)';
document.getElementById('drop').ondrop=e=>{e.preventDefault();
 document.getElementById('drop').style.borderColor='var(--line)';uploadAll([...e.dataTransfer.files]);};
async function uploadAll(files){const m=document.getElementById('upMsg');
 for(const f of files){m.className='status';m.textContent='Загрузка '+f.name+'…';
  const fd=new FormData();fd.append('file',f);
  try{await fetch('/api/fs/upload',{method:'POST',body:fd});
   m.className='status ok';m.textContent='Загружено: '+f.name;}
  catch(e){m.className='status err';m.textContent='Ошибка: '+e.message;}}
 loadFiles();}
</script></body></html>)HTMLPAGE";

#endif // INDEX_HTML_H
