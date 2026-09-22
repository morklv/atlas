/* Metric map ingestion, vehicle costmaps, A*, and local georeferencing.
   The same module runs in the browser and under Node for verification. */
(function(root,factory){if(typeof module==='object'&&module.exports)module.exports=factory();else root.SentinelNavigation=factory();})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const finite=Number.isFinite;
function positive(v,name){if(!finite(v)||v<=0)throw Error(name+' must be positive.');}
function parseXYZ(text){
  if(text.length>20*1024*1024)throw Error('Use a file under 20 MB.');
  const lines=text.replace(/^\uFEFF/,'').split(/\r?\n/);let start=0,count=null,columns=[0,1,2];
  if(lines[0].trim()==='ply'){
    if(!lines.slice(0,10).some(l=>l.trim()==='format ascii 1.0'))throw Error('Only ASCII PLY is supported. Export XYZ CSV or ASCII PLY.');
    let vertex=false,props=[],ended=false;
    for(let i=1;i<lines.length;i++){
      const t=lines[i].trim().split(/\s+/);
      if(t[0]==='element'){vertex=t[1]==='vertex';if(vertex)count=Number(t[2]);}
      if(vertex&&t[0]==='property'){if(t[1]==='list')throw Error('List vertex properties are unsupported.');props.push(t.at(-1));}
      if(t[0]==='end_header'){start=i+1;ended=true;break;}
    }
    columns=['x','y','z'].map(p=>props.indexOf(p));
    if(!ended||columns.some(c=>c<0)||!Number.isInteger(count)||count<3||count>250000)throw Error('PLY needs 3–250,000 vertices and x/y/z properties.');
  }else{
    while(start<lines.length&&(!lines[start].trim()||lines[start].trim().startsWith('#')))start++;
    const head=(lines[start]||'').trim().split(/[,;\s]+/).map(s=>s.replace(/^"|"$/g,'').toLowerCase());
    if(head.includes('x')&&head.includes('y')&&head.includes('z')){columns=['x','y','z'].map(p=>head.indexOf(p));start++;}
  }
  const points=[];
  for(let i=start;i<lines.length&&(count===null||points.length<count);i++){
    const row=lines[i].trim();if(!row||row.startsWith('#'))continue;
    const parts=row.split(/[,;\s]+/),p=columns.map(c=>parts[c]===undefined||parts[c]===''?NaN:Number(parts[c]));
    if(!p.every(finite))throw Error('Invalid numeric XYZ values on line '+(i+1)+'.');
    points.push(p);if(points.length>250000)throw Error('Use at most 250,000 points.');
  }
  if(points.length<3||(count!==null&&points.length!==count))throw Error('Point data is incomplete. At least three XYZ points are required.');
  return points;
}
function gridFromPoints(points,resolution=.5){
  positive(resolution,'Cell size');
  let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
  for(const p of points){minX=Math.min(minX,p[0]);minY=Math.min(minY,p[1]);maxX=Math.max(maxX,p[0]);maxY=Math.max(maxY,p[1]);}
  const n=Math.floor(Math.max(maxX-minX,maxY-minY)/resolution)+1;
  if(n<2||n>512)throw Error('Choose a cell size producing a grid between 2 and 512 cells per side.');
  const height=new Float64Array(n*n).fill(NaN),low=new Float64Array(n*n).fill(Infinity),known=new Uint8Array(n*n);
  for(const [x,y,z]of points){const ix=Math.floor((x-minX)/resolution),iy=Math.floor((y-minY)/resolution),i=iy*n+ix;
    height[i]=known[i]?Math.max(height[i],z):z;low[i]=Math.min(low[i],z);known[i]=1;}
  const roughness=Float64Array.from(height,(v,i)=>known[i]?v-low[i]:0);
  return {n,resolution,size:n*resolution,height,known,roughness,offset:[minX,minY],source:'imported point cloud',voxel:null};
}
function fromSurvey(data){const last=data.events.at(-1).map,n=Math.round(data.size/data.resolution);
  return {n,resolution:data.resolution,size:data.size,height:Float64Array.from(last.height),known:Uint8Array.from(last.sources,v=>v>0?1:0),roughness:new Float64Array(n*n),offset:[0,0],source:'synthetic completed survey',voxel:data.voxel};}
function inflate(base,shape,radius,resolution){
  if(!finite(radius)||radius<0||radius>3)throw Error('Robot radius must be between 0 and 3 m.');
  if(radius===0)return base.slice();const[nx,ny,nz]=shape,out=new Uint8Array(base.length),k=Math.ceil(radius/resolution+.5),offsets=[];
  for(let dz=nz===1?0:-k;dz<= (nz===1?0:k);dz++)for(let dy=-k;dy<=k;dy++)for(let dx=-k;dx<=k;dx++){
    const distance=Math.hypot(Math.max(0,Math.abs(dx)-.5)*resolution,Math.max(0,Math.abs(dy)-.5)*resolution,nz===1?0:Math.max(0,Math.abs(dz)-.5)*resolution);
    if(distance<=radius+1e-9)offsets.push([dx,dy,dz]);
  }
  for(let z=0;z<nz;z++)for(let y=0;y<ny;y++)for(let x=0;x<nx;x++){
    const i=(z*ny+y)*nx+x;if(!base[i])continue;let clear=true;
    for(const[dx,dy,dz]of offsets){const xx=x+dx,yy=y+dy,zz=z+dz;if(xx<0||yy<0||zz<0||xx>=nx||yy>=ny||zz>=nz||!base[(zz*ny+yy)*nx+xx]){clear=false;break;}}
    out[i]=clear?1:0;
  }return out;
}
function groundCostmap(grid,options={}){
  const o={radius:.2,maxHeight:2.5,maxStep:.3,maxSlope:25,...options},n=grid.n,r=grid.resolution;
  if(![o.maxHeight,o.maxStep,o.maxSlope].every(finite)||o.maxStep<=0||o.maxSlope<=0||o.maxSlope>=90)throw Error('Invalid terrain limits.');
  const base=new Uint8Array(n*n),reason=new Uint8Array(n*n);
  for(let i=0;i<base.length;i++){
    if(!grid.known[i]||!finite(grid.height[i])){reason[i]=1;continue;}
    if(o.roadOnly&&(!grid.semantic||grid.semantic[i]!==1)){reason[i]=5;continue;}
    if(grid.height[i]>o.maxHeight){reason[i]=2;continue;}
    if(grid.roughness[i]>o.maxStep){reason[i]=3;continue;}
    base[i]=1;
  }
  const passable=inflate(base,[n,n,1],o.radius,r);
  for(let i=0;i<base.length;i++)if(base[i]&&!passable[i])reason[i]=4;
  return{passable,reason,options:o,shape:[n,n,1],resolution:r};
}
function imageRoadCostmap(grid,options={}){
  if(!grid.semantic)throw Error('Image terrain routing requires semantic predictions.');
  const n=grid.n,o={roadBufferCells:1,...options};
  if(!Number.isInteger(o.roadBufferCells)||o.roadBufferCells<0||o.roadBufferCells>4)throw Error('Invalid road corridor width.');
  const passable=new Uint8Array(n*n),reason=new Uint8Array(n*n),penalty=new Float64Array(n*n);
  // Segmentation edges are coarse. Expand each road by one cell (or the chosen
  // width) so a continuous street remains usable when one predicted pixel drops.
  for(let y=0;y<n;y++)for(let x=0;x<n;x++){
    const i=y*n+x,label=grid.semantic[i];
    // Buildings, water and dense tree canopy do not establish vehicle
    // clearance from an overhead image. Open ground and low vegetation remain
    // usable at increasing cost so that rural tracks can join roads.
    if(label===2||label===3||label===5){reason[i]=label===2?2:label===3?6:7;continue;}
    let nearRoad=false;
    for(let dy=-o.roadBufferCells;dy<=o.roadBufferCells&&!nearRoad;dy++)for(let dx=-o.roadBufferCells;dx<=o.roadBufferCells;dx++){
      const xx=x+dx,yy=y+dy;if(xx>=0&&yy>=0&&xx<n&&yy<n&&grid.semantic[yy*n+xx]===1){nearRoad=true;break;}
    }
    passable[i]=1;
    // Background remains a costly connector. This closes gaps caused by a
    // coarse image label, while predicted road stays the preferred route.
    if(!nearRoad){reason[i]=label===4?8:5;penalty[i]=label===4?10:4;}
  }
  const component=new Int32Array(n*n).fill(-1),sizes=[];
  for(let first=0;first<passable.length;first++){
    if(!passable[first]||component[first]>=0)continue;
    const id=sizes.length,queue=[first];component[first]=id;let size=0;
    while(queue.length){const i=queue.pop();size++;const x=i%n,y=Math.floor(i/n);
      for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){const xx=x+dx,yy=y+dy,j=yy*n+xx;
        if(xx>=0&&yy>=0&&xx<n&&yy<n&&passable[j]&&component[j]<0){component[j]=id;queue.push(j);}}
    }sizes.push(size);
  }
  const primaryComponent=sizes.length?sizes.reduce((best,size,id)=>size>sizes[best]?id:best,0):-1;
  return{passable,reason,penalty,component,primaryComponent,options:o,shape:[n,n,1],resolution:grid.resolution};
}
function nearestPassable(costmap,point,maxDistance=8){
  const[nx,ny,nz]=costmap.shape,[x,y,z=0]=point;if(!Number.isInteger(x)||!Number.isInteger(y)||z!==0)throw Error('Invalid map point.');
  let best=null,bestDistance=Infinity;
  for(let yy=Math.max(0,y-maxDistance);yy<=Math.min(ny-1,y+maxDistance);yy++)for(let xx=Math.max(0,x-maxDistance);xx<=Math.min(nx-1,x+maxDistance);xx++){
    const i=(z*ny+yy)*nx+xx;if(!costmap.passable[i])continue;
    if(costmap.component&&costmap.primaryComponent>=0&&costmap.component[i]!==costmap.primaryComponent)continue;
    const distance=Math.hypot(xx-x,yy-y);
    if(distance<bestDistance){bestDistance=distance;best=[xx,yy,z];}
  }
  if(!best)throw Error('No predicted road corridor is near that point. Choose a street closer to the marker.');
  return{point:best,distance:bestDistance};
}
function airCostmap(grid,options={}){
  if(!grid.voxel)throw Error('An endpoint-only point cloud cannot establish free air. Aerial routing needs a ray-observed occupancy volume.');
  const[nz,ny,nx]=grid.voxel.shape,o={radius:.2,...options};
  const base=Uint8Array.from(grid.voxel.labels,v=>v===0?1:0);
  return {passable:inflate(base,[nx,ny,nz],o.radius,grid.resolution),shape:[nx,ny,nz],resolution:grid.resolution,options:o};
}
class Heap{
  constructor(){this.a=[];}
  push(item){const a=this.a;a.push(item);let i=a.length-1;while(i){const p=(i-1)>>1;if(a[p][0]<=item[0])break;a[i]=a[p];i=p;}a[i]=item;}
  pop(){const a=this.a,first=a[0],last=a.pop();if(a.length){let i=0;while(i*2+1<a.length){let c=i*2+1;if(c+1<a.length&&a[c+1][0]<a[c][0])c++;if(a[c][0]>=last[0])break;a[i]=a[c];i=c;}a[i]=last;}return first;}
}
function astar(costmap,start,goal,grid=null){
  const[nx,ny,nz]=costmap.shape,r=costmap.resolution,pass=costmap.passable;
  function idx(p){if(p.length!==3||p.some((v,i)=>!Number.isInteger(v)||v<0||v>=costmap.shape[i]))return-1;return(p[2]*ny+p[1])*nx+p[0];}
  const first=idx(start),last=idx(goal);
  if(first<0||last<0)throw Error('Start or destination is outside the mapped region.');
  if(!pass[first]||!pass[last])throw Error('Start or destination lacks sufficient observed clearance. Pick a lighter cell or survey that location.');
  const moves=nz===1?[[-1,0,0],[1,0,0],[0,-1,0],[0,1,0],[-1,-1,0],[-1,1,0],[1,-1,0],[1,1,0]]:[[-1,0,0],[1,0,0],[0,-1,0],[0,1,0],[0,0,-1],[0,0,1]];
  const g=new Float64Array(pass.length).fill(Infinity),parent=new Int32Array(pass.length).fill(-1),closed=new Uint8Array(pass.length),q=new Heap();
  const xyz=i=>[i%nx,Math.floor(i/nx)%ny,Math.floor(i/(nx*ny))];
  const heuristic=p=>Math.hypot(p[0]-goal[0],p[1]-goal[1],p[2]-goal[2])*r;
  g[first]=0;q.push([heuristic(start),first]);let visited=0;
  while(q.a.length){const[,i]=q.pop();if(closed[i])continue;closed[i]=1;visited++;if(i===last){
      const ids=[];for(let j=i;j!==-1;j=parent[j])ids.push(j);ids.reverse();
      const path=ids.map(j=>{const[x,y,z]=xyz(j);return[(x+.5)*r,(y+.5)*r,grid?grid.height[j]:(z+.5)*r];});
      let length=0;for(let j=1;j<path.length;j++)length+=Math.hypot(...path[j].map((v,k)=>v-path[j-1][k]));
      return{path,cost:g[i],length,visited,cellIds:ids};
    }
    const p=xyz(i);
    for(const[dx,dy,dz]of moves){const pp=[p[0]+dx,p[1]+dy,p[2]+dz],j=idx(pp);if(j<0||!pass[j]||closed[j])continue;
      // Diagonal traversal must not cut a blocked corner.
      if(dx&&dy&&(!pass[idx([p[0]+dx,p[1],p[2]])]||!pass[idx([p[0],p[1]+dy,p[2]])]))continue;
      let weight=Math.hypot(dx,dy,dz)*r;
      if(costmap.penalty)weight+=costmap.penalty[j]*r;
      if(grid){const rise=Math.abs(grid.height[j]-grid.height[i]),slope=Math.atan2(rise,weight)*180/Math.PI;
        if(rise>costmap.options.maxStep+1e-9||slope>costmap.options.maxSlope+1e-9)continue;
        weight+=2*rise;
      }
      const tentative=g[i]+weight;if(tentative<g[j]){g[j]=tentative;parent[j]=i;q.push([tentative+heuristic(pp),j]);}
    }
  }
  throw Error('No connected route exists under these limits. Unknown space, obstacles, or steep transitions separate the points. More survey data may be needed.');
}
function parseOrigin(text){
  const value=text.trim(),match=value.match(/@(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)/)||value.match(/(?:q=|query=)(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)/)||value.match(/^(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)$/);
  if(!match)throw Error('Paste latitude, longitude or a full map URL containing @latitude,longitude. Short links cannot supply coordinates offline.');
  const lat=Number(match[1]),lon=Number(match[2]);if(Math.abs(lat)>85||Math.abs(lon)>180)throw Error('Use latitude between −85 and 85 and longitude between −180 and 180.');return{lat,lon};
}
function localToLonLat(x,y,origin){
  // WGS84 local ENU -> ECEF -> geodetic. Z is omitted from GeoJSON because
  // imports do not establish an ellipsoid/orthometric altitude datum.
  const a=6378137,e2=6.69437999014e-3,lat=origin.lat*Math.PI/180,lon=origin.lon*Math.PI/180;
  const s=Math.sin(lat),c=Math.cos(lat),sl=Math.sin(lon),cl=Math.cos(lon),v=a/Math.sqrt(1-e2*s*s);
  const X=v*c*cl-sl*x-s*cl*y,Y=v*c*sl+cl*x-s*sl*y,Z=v*(1-e2)*s+c*y;
  const p=Math.hypot(X,Y);let phi=Math.atan2(Z,p*(1-e2));for(let i=0;i<8;i++){const nn=a/Math.sqrt(1-e2*Math.sin(phi)**2);phi=Math.atan2(Z+e2*nn*Math.sin(phi),p);}
  return[Math.atan2(Y,X)*180/Math.PI,phi*180/Math.PI];
}
return{parseXYZ,gridFromPoints,fromSurvey,groundCostmap,imageRoadCostmap,nearestPassable,airCostmap,astar,parseOrigin,localToLonLat};
});
