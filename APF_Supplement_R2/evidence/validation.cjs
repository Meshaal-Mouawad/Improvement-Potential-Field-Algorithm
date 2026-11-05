const {analyze}=require('./analyze.cjs');

function sha256(text) {
 const primes=[];for(let n=2;primes.length<64;n++){if(primes.every(p=>n%p))primes.push(n);}
 const frac=x=>(x-Math.floor(x))*4294967296>>>0, k=primes.map(p=>frac(Math.cbrt(p))), h=primes.slice(0,8).map(p=>frac(Math.sqrt(p)));
 const bytes=Array.from(unescape(encodeURIComponent(text)),c=>c.charCodeAt(0)), bitLength=bytes.length*8; bytes.push(128);
 while(bytes.length%64!==56)bytes.push(0);for(let i=7;i>=0;i--)bytes.push(i>=4?0:(bitLength>>> (8*i))&255);
 const ro=(x,n)=>(x>>>n)|(x<<(32-n));
 for(let off=0;off<bytes.length;off+=64){const w=[];for(let i=0;i<16;i++)w[i]=bytes.slice(off+4*i,off+4*i+4).reduce((s,x)=>(s<<8)|x,0);
 for(let i=16;i<64;i++){const x=w[i-15],y=w[i-2];w[i]=(w[i-16]+(ro(x,7)^ro(x,18)^(x>>>3))+w[i-7]+(ro(y,17)^ro(y,19)^(y>>>10)))|0;}
 let [a,b,c,d,e,f,g,z]=h;for(let i=0;i<64;i++){const t1=(z+(ro(e,6)^ro(e,11)^ro(e,25))+((e&f)^(~e&g))+k[i]+w[i])|0,t2=((ro(a,2)^ro(a,13)^ro(a,22))+((a&b)^(a&c)^(b&c)))|0;z=g;g=f;f=e;e=(d+t1)|0;d=c;c=b;b=a;a=(t1+t2)|0;}[a,b,c,d,e,f,g,z].forEach((v,i)=>h[i]=(h[i]+v)|0);}
 return h.map(v=>(v>>>0).toString(16).padStart(8,'0')).join('');
}
function canonical(value){if(Array.isArray(value))return '['+value.map(canonical).join(',')+']';if(value&&typeof value==='object')return '{'+Object.keys(value).sort().map(k=>JSON.stringify(k)+':'+canonical(value[k])).join(',')+'}';return JSON.stringify(value);}
function scalar(r){return Object.fromEntries(['map','method','gain','condition','tag','status','steps','length','minClear'].map(k=>[k,r[k]]));}

function validateData(rows,analysis,expected,formats={}) {
 const fail=m=>{throw Error(m);};
 if(rows.length!==expected.rowCount)fail('row count');
 const keys=new Set(),tags={},aggregates={};
 for(const r of rows){
  if(r.family!==undefined&&r.family!==r.map.split('_')[0])fail('family mismatch');
  if(r.split!==undefined&&r.split!==(r.tag==='calibration'?'development':'heldout'))fail('split mismatch');
  const key=[r.tag,r.map,r.method,r.condition,r.gain].join('/');
  if(keys.has(key))fail('duplicate key '+key);keys.add(key);
  if(!['success','collision','timeout','boundary','stall','nonfinite'].includes(r.status))fail('status '+key);
  if(!Number.isInteger(r.steps)||r.steps<0||!Number.isFinite(r.length)||!Number.isFinite(r.minClear))fail('numeric fields '+key);
  if(r.length<0)fail('negative length '+key);
  const step=r.tag==='heldout_fine'||r.tag==='posthoc_step_0.025'?.025:r.tag==='posthoc_step_0.0125'?.0125:.05;
  if(Math.abs(r.length-r.steps*step)>1e-9)fail('step length '+key);
  if(r.status==='collision'&&r.minClear>0)fail('positive collision clearance '+key);
  if(r.status==='success'&&r.minClear<=0)fail('nonpositive success clearance '+key);
  tags[r.tag]=(tags[r.tag]||0)+1;
  const k=r.tag+'/'+r.method+'/'+r.condition+'/'+r.status;aggregates[k]=(aggregates[k]||0)+1;
 }
 const targetTags={calibration:3600,heldout_frozen:2400,heldout_retuned:960,heldout_fine:192,'posthoc_step_0.025':96,'posthoc_step_0.0125':96};
 if(canonical(tags)!==canonical(targetTags))fail('tag counts');
 if(sha256(canonical(rows.map(scalar)))!==expected.scalarSha256)fail('baseline scalar SHA256 mismatch');
 if(canonical(aggregates)!==canonical(expected.aggregates))fail('expected aggregate mismatch');
 for(const [name,other]of Object.entries(formats))if(canonical(other)!==canonical(rows))fail('cross-format '+name);
 const recomputed=analyze(rows);
 if(canonical(analysis)!==canonical(recomputed))fail('analysis differs from deterministic recomputation');
 return {rows:rows.length,uniqueKeys:keys.size,baselineDigest:expected.scalarSha256,referenceContrasts:16,recoveryContrasts:2,crossFormats:Object.keys(formats)};
}
function validatePaths(experiment,rows) {
 const all=[...experiment.calibration,...experiment.heldout,...experiment.posthoc];
 if(all.length!==rows.length)throw Error('generated row count');
 let maxLengthDifference=0,maxClearanceDifference=0;
 all.forEach((r,i)=>{const target=rows[i];for(const k of ['map','method','gain','condition','tag','status','steps'])if(r[k]!==target[k])throw Error('generated '+k+' row '+i);for(const k of ['length','minClear'])if(!Number.isFinite(r[k])||Math.abs(r[k]-target[k])>1e-9)throw Error('generated '+k+' row '+i);maxLengthDifference=Math.max(maxLengthDifference,Math.abs(r.length-target.length));maxClearanceDifference=Math.max(maxClearanceDifference,Math.abs(r.minClear-target.minClear));});
 const maps={calibration:experiment.development.maps,test:experiment.test.maps};
 let segments=0;
 // Independent closest-point geometry: explicit dot product and clipping.
 function clearance(a,b,o){const vx=b[0]-a[0],vy=b[1]-a[1],d=vx*vx+vy*vy;let t=d===0?0:((o.c[0]-a[0])*vx+(o.c[1]-a[1])*vy)/d;t=Math.max(0,Math.min(1,t));return Math.sqrt((a[0]+t*vx-o.c[0])**2+(a[1]+t*vy-o.c[1])**2)-o.r-.15;}
 for(const r of all){
  const map=(r.tag==='calibration'?maps.calibration:maps.test).find(m=>m.id===r.map), path=r.path;
  if(!map||!Array.isArray(path)||path.length!==r.steps+1||Array.from(path).some(point=>!Array.isArray(point)||point.length!==2||!Number.isFinite(point[0])||!Number.isFinite(point[1]))||canonical(path[0])!==canonical(map.start))throw Error('path shape '+r.map);
  const step=r.tag==='heldout_fine'||r.tag==='posthoc_step_0.025'?.025:r.tag==='posthoc_step_0.0125'?.0125:.05;
  let length=0,minimum=Infinity;
  for(let i=1;i<path.length;i++){const a=path[i-1],b=path[i];segments++;
   if(b.some(v=>!Number.isFinite(v))||b[0]<.15||b[0]>9.85||b[1]<.15||b[1]>9.85)throw Error('invalid accepted position');
   const len=Math.hypot(b[0]-a[0],b[1]-a[1]);if(Math.abs(len-step)>1e-10)throw Error('step magnitude');length+=len;
   for(const o of map.obs){const c=clearance(a,b,o);minimum=Math.min(minimum,c);if(c<=0)throw Error('accepted collision');}
  }
  if(Math.abs(length-r.length)>1e-8)throw Error('path length');
  const end=path[path.length-1],goalDistance=Math.hypot(end[0]-map.goal[0],end[1]-map.goal[1]);
  if(r.status==='success'&&goalDistance>.1+1e-12)throw Error('false success');
  if(r.status==='timeout'&&goalDistance<=.1)throw Error('false timeout');
  if(r.status!=='collision'&&Math.abs(minimum-r.minClear)>1e-9)throw Error('path minimum clearance');
  if(r.status==='timeout'){const maximum=r.tag==='heldout_fine'?1200:r.tag==='posthoc_step_0.025'?1200:r.tag==='posthoc_step_0.0125'?2400:600;if(r.steps!==maximum)throw Error('premature timeout');}
 }
 return {trajectories:all.length,acceptedSegments:segments,maxLengthDifference,maxClearanceDifference,continuousReplayTolerance:1e-9,discreteReplayAgreement:'exact'};
}
if(typeof module!=='undefined')module.exports={validateData,validatePaths};


module.exports.canonical=canonical;
