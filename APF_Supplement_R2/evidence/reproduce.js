
function segmentDistance(a,b,c) {
 const dx=b[0]-a[0],dy=b[1]-a[1],q=dx*dx+dy*dy;
 const t=q?Math.max(0,Math.min(1,((c[0]-a[0])*dx+(c[1]-a[1])*dy)/q)):0;
 return Math.hypot(a[0]+t*dx-c[0],a[1]+t*dy-c[1]);
}
function collision(a,b,obs,robotRadius=0) {
 return obs.some(o=>segmentDistance(a,b,o.c)<=o.r+robotRadius);
}
function samples(o,n,phase=0,nonuniform=0) {
 // Angular warping preserves the circular boundary, with nonuniformity |a|<1.
 const theta=Array.from({length:n},(_,i)=>{let t=2*Math.PI*i/n; return t+phase+nonuniform*Math.sin(t);});
 return theta.map((t,i)=>{let prev=theta[(i+n-1)%n]-(i===0?2*Math.PI:0),next=theta[(i+1)%n]+(i===n-1?2*Math.PI:0);
 return {p:[o.c[0]+o.r*Math.cos(t),o.c[1]+o.r*Math.sin(t)],w:o.r*(next-prev)/2};});
}
function field(x,goal,obs,mode,gain,cfg,sampling) {
 let gx=goal[0]-x[0],gy=goal[1]-x[1],gd=Math.hypot(gx,gy);
 let fx=cfg.attraction*gx,fy=cfg.attraction*gy;
 for(const original of obs){
  const o={c:original.c,r:original.r+cfg.robotRadius};
  let list;
  if(mode==='nearest'){
   const dx=x[0]-o.c[0],dy=x[1]-o.c[1],d=Math.hypot(dx,dy);
   list=[{p:[o.c[0]+o.r*dx/(d||1),o.c[1]+o.r*dy/(d||1)],w:1}];
  }else { list=samples(o,sampling.n,sampling.facing?Math.atan2(o.c[1]-sampling.start[1],o.c[0]-sampling.start[0]):sampling.phase||0,sampling.nonuniform||0); if(sampling.duplicates>1) {const k=sampling.duplicates; list=Array.from({length:k},()=>list.map(z=>({p:z.p,w:z.w/k}))).flat();} }
  for(const z of list){
   const dx=x[0]-z.p[0],dy=x[1]-z.p[1],d=Math.hypot(dx,dy),clear=d;
   if(clear<cfg.influence && clear>0){
    const k=gain*(1/clear-1/cfg.influence)/(clear*clear*d);
    let weight=mode==='mean'?32/list.length:mode==='quadrature'?32*z.w/(2*Math.PI*o.r):1;
    fx+=weight*k*dx;fy+=weight*k*dy;
   }
  }
 }
 return [fx,fy];
}
function rollout(map,mode,gain,cfg,sampling){
 let p=map.start.slice(),path=[p],length=0,minClear=Infinity;
 for(let i=0;i<cfg.maxSteps;i++){
  const d=Math.hypot(p[0]-map.goal[0],p[1]-map.goal[1]);
  if(d<=cfg.goalTolerance) return {status:'success',steps:i,length,minClear,path};
  const f=field(p,map.goal,map.obs,mode,gain,cfg,sampling),mag=Math.hypot(...f);
  if(!Number.isFinite(mag)) return {status:'nonfinite',steps:i,length,minClear,path};
  if(mag<cfg.forceTolerance) return {status:'stall',steps:i,length,minClear,path};
  let step=Math.min(cfg.step,d),q=[p[0]+step*f[0]/mag,p[1]+step*f[1]/mag];
  if(q[0]<cfg.robotRadius||q[0]>10-cfg.robotRadius||q[1]<cfg.robotRadius||q[1]>10-cfg.robotRadius) return {status:'boundary',steps:i,length,minClear,path};
  for(const o of map.obs) minClear=Math.min(minClear,segmentDistance(p,q,o.c)-o.r-cfg.robotRadius);
  if(collision(p,q,map.obs,cfg.robotRadius)) return {status:'collision',steps:i,length,minClear,path};
  length+=step;p=q;path.push(p);
 }
 return {status:Math.hypot(p[0]-map.goal[0],p[1]-map.goal[1])<=cfg.goalTolerance?'success':'timeout',steps:cfg.maxSteps,length,minClear,path};
}

function rng(seed){let x=seed>>>0;return ()=>{x=(Math.imul(1664525,x)+1013904223)>>>0;return x/4294967296}}
function generate(seed,count){
 const rand=rng(seed),u=(a,b)=>a+(b-a)*rand(),maps=[],draws={single:0,gate:0,clutter:0};
 for(const family of ['single','gate','clutter']) for(let i=0;i<count;i++){
  let obs;
  do {draws[family]++;obs=[];
   if(family==='single') obs=[{c:[u(4,6),u(4.4,5.6)],r:u(.35,.65)}];
   if(family==='gate'){let x=u(4,6),r1=u(.35,.65),r2=u(.35,.65),gap=u(.25,.9),y=5+u(-.25,.25);obs=[{c:[x,y-(r1+.15+gap/2)],r:r1},{c:[x,y+(r2+.15+gap/2)],r:r2}];}
   if(family==='clutter') for(let j=0;j<6;j++)obs.push({c:[u(2.5,7.5),u(3,7)],r:u(.35,.65)});
  } while(obs.some((o,j)=>obs.slice(0,j).some(p=>Math.hypot(o.c[0]-p.c[0],o.c[1]-p.c[1])<o.r+p.r+.4))||obs.some(o=>o.c[0]-o.r-.15<.15||o.c[0]+o.r+.15>9.85||o.c[1]-o.r-.15<.15||o.c[1]+o.r+.15>9.85)||[[[.75,5],[.75,.5]],[[.75,.5],[9.25,.5]],[[9.25,.5],[9.25,5]]].some(seg=>collision(seg[0],seg[1],obs,.15)));
  maps.push({id:family+'_'+i,family,start:[.75,5],goal:[9.25,5],obs});
 } return {maps,draws};
}
const cfg={attraction:1,robotRadius:.15,influence:1,step:.05,goalTolerance:.1,maxSteps:600,forceTolerance:1e-10};
const methods=['raw','mean','quadrature','nearest'], gains=[.003,.01,.03,.1,.3,1];
const conditions=[{id:'u16',n:16},{id:'reference',n:32},{id:'u64',n:64},{id:'phase',n:32,phase:Math.PI/32},{id:'nonuniform',n:32,nonuniform:.8,facing:true}];
function run(map,m,g,c,config=cfg){return rollout(map,m,g,config,{...c,start:map.start});}
function select(rows){let a=gains.map(g=>{let r=rows.filter(x=>x.gain===g);return {g,success:r.filter(x=>x.status==='success').length,cost:r.reduce((s,x)=>s+(x.status==='success'?x.length:30),0)};});a.sort((x,y)=>y.success-x.success||x.cost-y.cost||x.g-y.g);return a[0];}
function row(map,method,gain,cond,result,tag){return {map:map.id,family:map.family,method,gain,condition:cond.id,tag,...result};}

const dev=generate(20261104,10), calibration=[];
for(const m of methods)for(const c of conditions)for(const g of gains)for(const map of dev.maps)
 calibration.push(row(map,m,g,c,run(map,m,g,c),'calibration'));
const selections={};
for(const m of methods){selections[m]={};for(const c of conditions){
 const group=calibration.filter(r=>r.method===m&&r.condition===c.id);
 for(const g of gains){const rr=group.filter(r=>r.gain===g);if(rr.length!==30||new Set(rr.map(x=>x.map)).size!==30)throw Error('incomplete group');}
 selections[m][c.id]=select(group);
}}
const test=generate(20261105,40),heldout=[],posthoc=[];
for(const map of test.maps)for(const m of methods)for(const c of conditions)
 heldout.push(row(map,m,selections[m].reference.g,c,run(map,m,selections[m].reference.g,c),'heldout_frozen'));
for(const map of test.maps)for(const m of ['raw','quadrature'])for(const c of conditions.filter(x=>x.id!=='reference'))
 heldout.push(row(map,m,selections[m][c.id].g,c,run(map,m,selections[m][c.id].g,c),'heldout_retuned'));
for(const map of test.maps.filter(x=>+x.id.split('_')[1]<8))for(const m of methods)for(const c of conditions.filter(x=>x.id==='reference'||x.id==='nonuniform'))
 heldout.push(row(map,m,selections[m].reference.g,c,run(map,m,selections[m].reference.g,c,{...cfg,step:.025,maxSteps:1200}),'heldout_fine'));
for(const map of test.maps.filter(x=>+x.id.split('_')[1]<8))for(const m of methods)for(const step of [.025,.0125]){
 const c=conditions[0];posthoc.push(row(map,m,selections[m].reference.g,c,run(map,m,selections[m].reference.g,c,{...cfg,step,maxSteps:Math.round(30/step)}),'posthoc_step_'+step));
}
console.log(JSON.stringify({development:dev,test,calibration,selections,heldout,posthoc}));
