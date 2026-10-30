import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
import {createRequire} from 'node:module';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../../../..');
const require=createRequire(path.join(root,'build/research-20261012/commercial/rea-cli/package.json'));
const {parse}=require('@babel/parser');
const base=path.join(root,'build/research-20261012/xr_reference-binary');
const filename=path.join(root,'assets/references/xr_reference/analysis-runtime/standalone/xr-slam.js');
const bytes=fs.readFileSync(filename),source=bytes.toString('utf8'),ast=parse(source,{sourceType:'module'});
const calls=[],properties=[],largeLiterals=[];let nodes=0;
const stack=[ast];
while(stack.length){const node=stack.pop();if(!node||typeof node!=='object')continue;nodes++;
  if(node.type==='CallExpression'&&node.callee?.type==='MemberExpression'){
    const property=node.callee.property;const name=property?.name||property?.value;
    if(typeof name==='string'&&name.startsWith('_c8EmAsm_'))calls.push({name,start:node.start,end:node.end,line:node.loc.start.line,source:source.slice(node.start,node.end),arguments:node.arguments.map(arg=>source.slice(arg.start,arg.end))});
  }
  if(node.type==='ObjectProperty'&&['updateDeviceMotion','setEventQueue'].includes(node.key?.name||node.key?.value))properties.push({name:node.key.name||node.key.value,start:node.start,end:node.end,source:source.slice(node.start,node.end)});
  if(node.type==='StringLiteral'&&node.value.startsWith('data:application/')&&node.value.includes(';base64,'))largeLiterals.push({start:node.start,end:node.end,characters:node.value.length,source_sha256:createHash('sha256').update(node.value).digest('hex')});
  for(const [key,value] of Object.entries(node)){if(['loc','extra','tokens','comments'].includes(key))continue;if(Array.isArray(value))stack.push(...value);else if(value&&typeof value==='object')stack.push(value);}
}
const report={schema:'xr_reference-actual-js-glue-boundaries-v1',input:filename,input_sha256:createHash('sha256').update(bytes).digest('hex'),parser:'existing REA @babel/parser',ast_nodes:nodes,calls:calls.sort((a,b)=>a.start-b.start),properties,embedded_literals:largeLiterals,scope:'Original distributed JS boundaries. No proprietary C++ solver source reconstructed.'};
fs.writeFileSync(path.join(base,'js-glue-boundaries.json'),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({ast_nodes:nodes,wasmCalls:calls.length,boundaryProperties:properties.map(p=>p.name),embeddedLiterals:largeLiterals.length},null,2));
