'use strict';
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict'),crypto=require('node:crypto');
const {validateData,validatePaths}=require('./validation.cjs'),base=__dirname,read=f=>fs.readFileSync(path.join(base,f),'utf8'),rows=JSON.parse(read('metrics.json')),analysis=JSON.parse(read('statistics_r1.json')),expected=JSON.parse(read('expected_r1.json'));
const hashes=()=>Object.fromEntries(['metrics.json','metrics.jsonl','metrics.csv','paired_statistics.json','statistics_r1.json','maps_and_gains.json'].map(f=>[f,crypto.createHash('sha256').update(read(f)).digest('hex')]));
const before=hashes(),clone=x=>JSON.parse(JSON.stringify(x)),results=[];
for(const name of ['outcome','coordinated_outcome','coherent_outcome','duplicate_key','gain','statistic','transition','cross_format']){
 const r=clone(rows),a=clone(analysis);let formats={jsonl:clone(rows)};
 if(['outcome','coordinated_outcome','coherent_outcome'].includes(name)){const z=r.find(x=>x.status==='success');z.status='collision';if(name==='coherent_outcome')z.minClear=-.001;if(name!=='outcome')formats={csv:clone(r),jsonl:clone(r)};}
 if(name==='duplicate_key')r[1]=clone(r[0]);if(name==='gain')r[0].gain=.123;
 if(name==='statistic')a.referenceContrasts['raw/u16'].ci[0]+=.01;
 if(name==='transition')a.recovery['countnorm_vs_raw/u16'].transitions.collision.success++;
 if(name==='cross_format')formats.jsonl[0].steps++;
 let error;try{validateData(r,a,expected,formats);}catch(e){error=e.message;}
 assert.ok(error,'validator accepted '+name);results.push({name,rejected:true,reason:error});
}
const replayRows=[{map:'probe_0',method:'raw',gain:.01,condition:'u16',tag:'calibration',status:'success',steps:1,length:.05,minClear:3.55}];
for(const name of ['malformed_waypoint','nonfinite_replay_length','nonfinite_replay_clearance']){
 const e={development:{maps:[{id:'probe_0',start:[.75,5],goal:[.8,5],obs:[{c:[5,5],r:.5}]}]},test:{maps:[]},calibration:[{...replayRows[0],path:[[.75,5],[.8,5]]}],heldout:[],posthoc:[]};
 assert.doesNotThrow(()=>validatePaths(e,replayRows),'unmodified structural probe');
 if(name==='malformed_waypoint')e.calibration[0].path[1]=[.8];
 if(name==='nonfinite_replay_length')e.calibration[0].length=NaN;
 if(name==='nonfinite_replay_clearance')e.calibration[0].minClear=NaN;
 let error;try{validatePaths(e,replayRows);}catch(err){error=err.message;}
 assert.ok(error,'validator accepted '+name);results.push({name,rejected:true,reason:error});
}
const os=require('node:os'),cp=require('node:child_process'),temp=fs.mkdtempSync(path.join(os.tmpdir(),'apf-negative-')),cli=[];
try{
 fs.cpSync(base,path.join(temp,'evidence'),{recursive:true});const t=path.join(temp,'evidence');
 for(const name of ['coordinated_coherent_outcome','statistic']){
  const changes={};
  if(name==='coordinated_coherent_outcome'){
   const r=clone(rows),z=r.find(x=>x.status==='success');z.status='collision';z.minClear=-.001;
   changes['metrics.json']=JSON.stringify(r);changes['metrics.jsonl']=r.map(x=>JSON.stringify(x)).join('\n');
   const columns=Object.keys(r[0]);changes['metrics.csv']=[columns.join(','),...r.map(x=>columns.map(k=>x[k]).join(','))].join('\n');
  }else{const a=clone(analysis);a.recovery['countnorm_vs_raw/u16'].ci[0]+=.01;changes['statistics_r1.json']=JSON.stringify(a);}
  const manifest=JSON.parse(read('integrity_r1.json'));
  for(const [n,s]of Object.entries(changes)){fs.writeFileSync(path.join(t,n),s);manifest[n]=crypto.createHash('sha256').update(s).digest('hex');}
  fs.writeFileSync(path.join(t,'integrity_r1.json'),JSON.stringify(manifest));
  const run=cp.spawnSync(process.execPath,[path.join(t,'verify.cjs')],{encoding:'utf8'});assert.equal(run.status,1,'CLI accepted '+name);
  cli.push({name,exitStatus:run.status,reason:run.stderr.trim()});
  for(const n of Object.keys(changes))fs.copyFileSync(path.join(base,n),path.join(t,n));
 }
}finally{fs.rmSync(temp,{recursive:true,force:true});}
assert.deepEqual(hashes(),before,'validator modified evidence');
console.log(JSON.stringify({negativeControls:results,cliNegativeControls:cli,evidenceUnchanged:true},null,2));

