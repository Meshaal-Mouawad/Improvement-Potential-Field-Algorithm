'use strict';
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto');
const {analyze}=require('./analyze.cjs'),{validateData,validatePaths,canonical}=require('./validation.cjs');
const base=__dirname,read=name=>fs.readFileSync(path.join(base,name),'utf8'),json=name=>JSON.parse(read(name));
function near(a,b,where='statistics') {
 if(typeof a==='number'&&typeof b==='number'){if(Math.abs(a-b)>1e-14)throw Error(where);return;}
 if(Array.isArray(a)&&Array.isArray(b)){if(a.length!==b.length)throw Error(where);a.forEach((v,i)=>near(v,b[i],where+'/'+i));return;}
 if(a&&b&&typeof a==='object'&&typeof b==='object'){if(canonical(Object.keys(a).sort())!==canonical(Object.keys(b).sort()))throw Error(where+' keys');Object.keys(a).forEach(k=>near(a[k],b[k],where+'/'+k));return;}
 if(a!==b)throw Error(where);
}
function legacyStatistics(rows,a){
 const fields=['difference','ci','pairedN','meanLengthDifference','meanClearanceDifference'],paired={};
 for(const [k,v] of Object.entries(a.referenceContrasts))paired[k]=Object.fromEntries(fields.map(f=>[f,v[f]]));
 const statuses=['success','collision','timeout','boundary','stall','nonfinite'],count=rr=>Object.fromEntries(statuses.map(s=>[s,rr.filter(r=>r.status===s).length]));
 const sum={},trans={},family={},posthoc={};
 for(const r of rows.filter(r=>r.tag.startsWith('heldout'))){const k=[r.tag,r.method,r.condition].join('/');if(!sum[k]){const rr=rows.filter(x=>[x.tag,x.method,x.condition].join('/')===k);sum[k]={n:rr.length,...count(rr)};}}
 for(const [k,v]of Object.entries(a.referenceContrasts))trans[k]={lost:v.losses,gained:v.gains};
 for(const r of rows.filter(r=>r.tag==='heldout_frozen')){const k=[r.method,r.condition,r.family].join('/');if(!family[k])family[k]=count(rows.filter(x=>x.tag==='heldout_frozen'&&[x.method,x.condition,x.family].join('/')===k));}
 for(const r of rows.filter(r=>r.tag.startsWith('posthoc'))){const k=r.method+'/'+r.tag;if(!posthoc[k])posthoc[k]=count(rows.filter(x=>x.method+'/'+x.tag===k));}
 for(const m of ['raw','mean','quadrature','nearest'])posthoc[m+'/baseline']=count(rows.filter(x=>x.tag==='heldout_frozen'&&x.method===m&&x.condition==='u16'&&Number(x.map.split('_')[1])<8));
 return {paired,summary:{sum,trans},family,posthoc};
}
function checkLegacy(rows,a,old){near(old,legacyStatistics(rows,a),'legacy statistics');}
function verify(args=process.argv.slice(2)){
 const hashes=json('integrity_r1.json');
 for(const [file,hash]of Object.entries(hashes)){const got=crypto.createHash('sha256').update(fs.readFileSync(path.join(base,file))).digest('hex');if(got!==hash)throw Error('file integrity '+file);}
 const rows=json('metrics.json'),lines=read('metrics.jsonl').trim().split(/\r?\n/).map(s=>JSON.parse(s)),csv=read('metrics.csv').trim().split(/\r?\n/),header=csv.shift().split(',');
 const csvRows=csv.map(line=>Object.fromEntries(line.split(',').map((v,i)=>[header[i],['gain','steps','length','minClear'].includes(header[i])?Number(v):v])));
 const statistics=json('statistics_r1.json'),result=validateData(rows,statistics,json('expected_r1.json'),{csv:csvRows,jsonl:lines});
 const statFlag=args.indexOf('--statistics');if(statFlag!==-1){if(!args[statFlag+1])throw Error('missing statistics filename');validateData(rows,JSON.parse(fs.readFileSync(args[statFlag+1],'utf8')),json('expected_r1.json'));result.externalStatistics=true;}
 checkLegacy(rows,statistics,json('paired_statistics.json'));
 const flag=args.indexOf('--trajectories');
 if(flag!==-1){if(!args[flag+1])throw Error('missing trajectory filename');const e=JSON.parse(fs.readFileSync(args[flag+1],'utf8')),maps=json('maps_and_gains.json');
  near(e.development.maps,maps.development,'development maps');near(e.test.maps,maps.test,'test maps');near(e.selections,maps.selections,'selections');
  near({development:e.development.draws,test:e.test.draws},maps.draws,'generator rejection draws');
  result.trajectories=validatePaths(e,rows);
 }
 return {...result,integrityFiles:Object.keys(hashes).length,legacyStatistics:'all paired,summary,family,posthoc verified',readOnly:true};
}
if(require.main===module){try{console.log(JSON.stringify(verify(),null,2));}catch(e){console.error('VALIDATION FAILED: '+e.message);process.exitCode=1;}}
module.exports={verify,checkLegacy,legacyStatistics};

