import {createServer} from 'node:http';
import {readFile,stat} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.mjs':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.json':'application/json','.wasm':'application/wasm','.png':'image/png','.txt':'text/plain; charset=utf-8'};
export async function startServer({root='dist',base='/',port=8000,host='127.0.0.1'}={}){
  if(!/^\/(?:[a-zA-Z0-9_-]+\/)*$/.test(base))throw new Error('Base must be a path with leading and trailing slashes');
  const directory=path.resolve(root);
  const server=createServer(async(req,res)=>{
    const url=new URL(req.url,'http://localhost');
    if(base!=='/'&&url.pathname===base.slice(0,-1)){res.writeHead(301,{Location:base});res.end();return;}
    if(!url.pathname.startsWith(base)){res.writeHead(404);res.end('Not found');return;}
    try{
      const relative=decodeURIComponent(url.pathname.slice(base.length))||'index.html';
      const file=path.resolve(directory,relative);
      if(!file.startsWith(directory+path.sep)||!(await stat(file)).isFile()){res.writeHead(404);res.end('Not found');return;}
      const bytes=await readFile(file);
      res.writeHead(200,{'Content-Type':types[path.extname(file)]||'application/octet-stream','Cache-Control':'no-cache'});
      res.end(req.method==='HEAD'?undefined:bytes);
    }catch{res.writeHead(404);res.end('Not found');}
  });
  await new Promise(resolve=>server.listen(port,host,resolve));
  const address=server.address();
  return{server,url:`http://${host}:${address.port}${base}`};
}
if(process.argv[1]&&path.resolve(process.argv[1])===fileURLToPath(import.meta.url)){
  const args=process.argv.slice(2),value=(flag,fallback)=>args.includes(flag)?args[args.indexOf(flag)+1]:fallback;
  const {url}=await startServer({base:value('--base','/'),port:Number(value('--port','8000')),host:value('--host','127.0.0.1')});
  console.log(`MicroDuck preview: ${url}`);
}
