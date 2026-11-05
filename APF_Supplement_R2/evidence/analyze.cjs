'use strict';
// Input rows use map,method,gain,condition,tag,status,steps,length,minClear.
// Deterministic statistical analysis; no simulation or outcome mutation.
function analyze(rows) {
 const families=['single','gate','clutter'], methods=['raw','mean','quadrature','nearest'], conditions=['u16','u64','phase','nonuniform'];
 const ids=families.flatMap(f=>Array.from({length:40},(_,i)=>f+'_'+i));
 function group(tag,method,condition) {
  const a=rows.filter(r=>r.tag===tag&&r.method===method&&r.condition===condition), by=new Map(a.map(r=>[r.map,r]));
  if(a.length!==120||by.size!==120||ids.some(id=>!by.has(id))) throw Error('Incomplete paired group: '+[tag,method,condition].join('/'));
  return ids.map(id=>by.get(id));
 }
 let state=20261106;
 const rand=()=>{state=(Math.imul(1664525,state)+1013904223)>>>0; return state/4294967296;};
 // One common draw array: replicate, family single/gate/clutter, then 40 draws.
 const draws=Array.from({length:10000},()=>families.flatMap((f,j)=>Array.from({length:40},()=>j*40+Math.floor(rand()*40))));
 const percentile=(sorted,p)=>sorted[Math.ceil(p*sorted.length)-1];
 function contrast(a,b) {
  const differences=a.map((r,i)=>Number(b[i].status==='success')-Number(r.status==='success'));
  const boot=draws.map(draw=>draw.reduce((s,i)=>s+differences[i],0)/120).sort((a,b)=>a-b);
  const both=a.map((r,i)=>[r,b[i]]).filter(([r,s])=>r.status==='success'&&s.status==='success');
  const statuses=['success','collision','timeout','boundary','stall','nonfinite'];
  const transitions=Object.fromEntries(statuses.map(s=>[s,Object.fromEntries(statuses.map(t=>[t,0]))]));
  a.forEach((r,i)=>{transitions[r.status][b[i].status]++;});
  return {difference:differences.reduce((a,b)=>a+b,0)/120,ci:[percentile(boot,.025),percentile(boot,.975)],
   pairedN:both.length,meanLengthDifference:both.length?both.reduce((s,[r,t])=>s+(t.length-r.length),0)/both.length:null,
   meanClearanceDifference:both.length?both.reduce((s,[r,t])=>s+(t.minClear-r.minClear),0)/both.length:null,
   gains:differences.filter(x=>x===1).length,losses:differences.filter(x=>x===-1).length,transitions};
 }
 const referenceContrasts={};
 for(const m of methods)for(const c of conditions) referenceContrasts[m+'/'+c]=contrast(group('heldout_frozen',m,'reference'),group('heldout_frozen',m,c));
 const recovery={
  'countnorm_vs_raw/u16':contrast(group('heldout_frozen','raw','u16'),group('heldout_frozen','mean','u16')),
  'recalibratedraw_vs_frozenraw/u16':contrast(group('heldout_frozen','raw','u16'),group('heldout_retuned','raw','u16'))
 };
 return {protocol:{replicates:10000,seed:20261106,rng:'LCG32: state=(1664525*state+1013904223) mod 2^32; u=state/2^32',
 strata:families,drawsPerStratum:40,order:'replicate, family, draw; numeric map suffix within family; same draws for every contrast',
 interval:'two-sided 95% percentile; nearest rank ceil(p*10000)-1 zero-based at p=.025,.975',unit:'success proportion; multiply differences and limits by 100 for percentage points',
 continuous:'mean paired difference over maps successful under both arms'},referenceContrasts,recovery};
}
if(typeof module!=='undefined') module.exports={analyze};

if(require.main===module){const fs=require('node:fs'),path=require('node:path');const args=process.argv.slice(2),file=args.find(a=>!a.startsWith('--'))||path.join(__dirname,'metrics.json'),rows=JSON.parse(fs.readFileSync(file,'utf8')),a=analyze(rows);console.log(JSON.stringify(args.includes('--legacy')?require('./verify.cjs').legacyStatistics(rows,a):a,null,2));}

